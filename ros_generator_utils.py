# -*- coding: utf-8 -*-
import os
import json
import logging
import getpass
import platform
import subprocess
import stat
import re
import time
from typing import Any, Dict, List, Optional
from langchain_core.chat_history import InMemoryChatMessageHistory
from langchain_core.messages import HumanMessage, AIMessage
from langchain_community.callbacks.manager import get_openai_callback
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
import tiktoken

"""Common utility class for ROS code generator"""
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Define LLM API URL and key
# using deepseek api
API_URL = "https://api.deepseek.com/v1"
model = "deepseek-v4"
temperature = 0.2

# # Using qwen API
# API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
# # model = "qwen3-coder-plus"
# model = "qwen3.6"
# temperature = 0.2

# # Using glm API
# API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
# model = "glm-5.1"
# temperature = 0.2

# Avoid indefinite hangs when the API connection stalls (no default read timeout in client).
LLM_REQUEST_TIMEOUT = 240.0
LLM_MAX_RETRIES = 4
LLM_CALL_MAX_ATTEMPTS = 3
LLM_CALL_RETRY_SLEEP_S = 2.0

# Third-party model ids (e.g. Qwen) are not in tiktoken's model map; cl100k_base is a fine local estimate.
def approx_token_count(text: str, model_name: str) -> int:
    """Approximate token count for logging only (not billing)."""
    try:
        lower = (model_name or "").lower()
        if lower.startswith(("qwen", "glm")) or "deepseek" in lower:
            enc = tiktoken.get_encoding("cl100k_base")
        else:
            try:
                enc = tiktoken.encoding_for_model(model_name)
            except Exception:
                enc = tiktoken.get_encoding("cl100k_base")
        return len(enc.encode(text))
    except Exception:
        return len(re.findall(r"\w+|[^\w\s]", text))

# # Using kim API
# API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
# model = "glm-4.6"
# temperature = 0.2

class ROSGeneratorUtils:
    """Common utility class for ROS code generator"""
    
    def __init__(self, output_dir: str):
        """Initialize the utility class
        
        Args:
            output_dir: Output directory
        """
        self.output_dir = output_dir
        # Initialize conversation memory
        self.memory_dir = os.path.join(output_dir, "memory")
        self.memory = None

        # Token usage accumulators (used by caller for stats)
        self.total_prompt_tokens = 0
        self.total_completion_tokens = 0
        self.total_tokens = 0
        
        # Initialize logger
        self.logger = logging.getLogger(__name__)

    def reset_token_stats(self):
        """Reset token usage accumulators."""
        self.total_prompt_tokens = 0
        self.total_completion_tokens = 0
        self.total_tokens = 0
    
    def initialize_memory(self):
        """Initialize conversation memory (langchain_core InMemoryChatMessageHistory; avoids deprecated BufferMemory)."""
        self.memory = InMemoryChatMessageHistory()
        self.logger.info("Conversation memory initialized")
    
    def save_memory(self, component_name: str):
        """Save conversation memory to combined file
        
        Args:
            component_name: Component name used to generate memory file name
        """
        if self.memory is None:
            self.logger.warning("Conversation memory unavailable, cannot save")
            return
        
        try:
            # Get messages from memory
            messages = self.memory.messages
            
            # Convert messages to serializable format
            serialized_messages = []
            for msg in messages:
                serialized_messages.append({
                    "type": msg.type,
                    "content": msg.content
                })
            
            # Save directly to combined memory file
            self.save_to_combined_memory(component_name, serialized_messages)
        except Exception as e:
            self.logger.error(f"Error saving conversation memory: {str(e)}")
            
    def save_to_combined_memory(self, component_name: str, serialized_messages: list):
        """Save component memory to combined memory file
        
        Args:
            component_name: Component name
            serialized_messages: Serialized message list
        """
        try:
            # Lazily create memory directory only when we actually persist memory.
            os.makedirs(self.memory_dir, exist_ok=True)
            # Combined memory file path
            combined_memory_file = os.path.join(self.memory_dir, "combined_memory.json")
            
            # Read existing combined memory file if it exists
            combined_memory = {}
            if os.path.exists(combined_memory_file):
                with open(combined_memory_file, 'r', encoding='utf-8') as f:
                    combined_memory = json.load(f)
            
            # Update component memory
            combined_memory[component_name] = serialized_messages
            
            # Save updated combined memory
            with open(combined_memory_file, 'w', encoding='utf-8') as f:
                json.dump(combined_memory, f, ensure_ascii=False, indent=2)
                
            self.logger.info(f"Saved memory for component {component_name} to combined memory file")
        except Exception as e:
            self.logger.error(f"Error saving to combined memory file: {str(e)}")
    
    def load_from_combined_memory(self) -> bool:
        """Load conversation memory from combined memory file
        
        Returns:
            bool: Whether memory was loaded successfully
        """
        try:
            # Combined memory file path
            combined_memory_file = os.path.join(self.memory_dir, "combined_memory.json")
            
            # Check if the file exists
            if not os.path.exists(combined_memory_file):
                self.logger.info("Combined memory file does not exist, will try to load from individual files")
                return False
            
            # Load combined memory from file
            with open(combined_memory_file, 'r', encoding='utf-8') as f:
                combined_memory = json.load(f)
            
            # Clear current memory
            self.memory.clear()
            
            # Recreate messages and add to memory
            # Select memory of the first component
            if combined_memory:
                first_component = next(iter(combined_memory.keys()))
                serialized_messages = combined_memory[first_component]
                for msg in serialized_messages:
                    if msg["type"] == "human":
                        self.memory.add_message(HumanMessage(content=msg["content"]))
                    elif msg["type"] == "ai":
                        self.memory.add_message(AIMessage(content=msg["content"]))
                self.logger.info(f"Loaded conversation memory for first component {first_component} from combined memory file")
            else:
                self.logger.warning("Combined memory file is empty")
                return False
                
        except Exception as e:
            self.logger.error(f"Error loading memory from combined memory file: {str(e)}")
            return False

    @staticmethod
    def _history_messages_with_content(messages: List[Any]) -> List[Any]:
        """Drop empty history entries that break LangChain message serialization."""
        out: List[Any] = []
        for msg in messages or []:
            c = getattr(msg, "content", None)
            if isinstance(c, str) and c.strip():
                out.append(msg)
        return out

    @staticmethod
    def _extract_langchain_content(response: Any) -> str:
        """Normalize AIMessage / ChatResult content (incl. empty or reasoning-only payloads)."""
        if response is None:
            return ""
        content = getattr(response, "content", None)
        if isinstance(content, str):
            text = content.strip()
            if text:
                return content
        extra = getattr(response, "additional_kwargs", None) or {}
        if isinstance(extra, dict):
            for key in ("content", "reasoning_content"):
                v = extra.get(key)
                if isinstance(v, str) and v.strip():
                    return v
        if content is not None and not isinstance(content, str):
            return str(content).strip()
        return ""

    def _invoke_langchain_once(
        self,
        llm: ChatOpenAI,
        prompt: str,
        should_load_memory: bool,
    ) -> Any:
        """Single LLM invoke; on chain failure, fall back to direct ``llm.invoke``."""
        if should_load_memory and self.memory is not None:
            self.load_from_combined_memory()
            history = self._history_messages_with_content(self.memory.messages)
            try:
                chain = ChatPromptTemplate.from_messages(
                    [MessagesPlaceholder(variable_name="history"), ("human", "{input}")]
                ) | llm
                return chain.invoke({"input": prompt, "history": history})
            except Exception as chain_err:
                self.logger.warning(
                    "LangChain history chain failed (%s); retrying with direct invoke",
                    chain_err,
                )
        return llm.invoke([HumanMessage(content=prompt)])

    def call_langchain(
        self,
        prompt: str,
        api_key: str = None,
        component_name: str = None,
        use_memory: bool = False,
        save_memory: Optional[bool] = None,
        load_memory: Optional[bool] = None,
    ) -> str:
        """
        Generate code using LangChain, record token usage, and support conversation memory.

        Memory mode (first match wins): if ``save_memory`` is not None → never load, save iff True;
        elif ``load_memory`` is not None → load iff True, never save; else both follow ``use_memory``.
        """
        if api_key is None:
            self.logger.warning("API key not provided")
            return None

        if save_memory is not None:
            should_load_memory, should_save_memory = False, save_memory
        elif load_memory is not None:
            should_load_memory, should_save_memory = load_memory, False
        else:
            should_load_memory = should_save_memory = bool(use_memory)

        llm = ChatOpenAI(
            model_name=model,
            temperature=temperature,
            openai_api_key=api_key,
            base_url=API_URL,
            timeout=LLM_REQUEST_TIMEOUT,
            max_retries=LLM_MAX_RETRIES,
        )

        last_err: Optional[Exception] = None
        for attempt in range(1, LLM_CALL_MAX_ATTEMPTS + 1):
            try:
                self.logger.info(
                    "[Token Stats] API input tokens (approx): %s (attempt %s/%s)",
                    approx_token_count(prompt, model),
                    attempt,
                    LLM_CALL_MAX_ATTEMPTS,
                )
                self.logger.info(
                    "Component %s: memory load=%s save=%s attempt=%s/%s",
                    component_name,
                    should_load_memory,
                    should_save_memory,
                    attempt,
                    LLM_CALL_MAX_ATTEMPTS,
                )
                with get_openai_callback() as cb:
                    response = self._invoke_langchain_once(llm, prompt, should_load_memory)
                self.logger.info(
                    "[Token Stats] LangChain: prompt=%s completion=%s total=%s",
                    cb.prompt_tokens,
                    cb.completion_tokens,
                    cb.total_tokens,
                )
                self.total_prompt_tokens += cb.prompt_tokens
                self.total_completion_tokens += cb.completion_tokens
                self.total_tokens += cb.total_tokens

                content = self._extract_langchain_content(response)
                if not (content or "").strip():
                    raise ValueError("LLM returned empty content")

                if self.memory is not None:
                    self.memory.add_message(HumanMessage(content=prompt))
                    self.memory.add_message(AIMessage(content=content))
                    if should_save_memory and component_name:
                        if not should_load_memory:
                            self.memory.clear()
                            self.memory.add_message(HumanMessage(content=prompt))
                            self.memory.add_message(AIMessage(content=content))
                        self.save_memory(component_name)
                        self.logger.info(
                            "Save current conversation memory for component %s",
                            component_name,
                        )

                self.logger.info(
                    "[Token Stats] API output tokens (approx): %s",
                    approx_token_count(content, model),
                )
                return content
            except Exception as e:
                last_err = e
                self.logger.warning(
                    "LangChain call failed for %s (attempt %s/%s): %s",
                    component_name,
                    attempt,
                    LLM_CALL_MAX_ATTEMPTS,
                    e,
                )
                if attempt < LLM_CALL_MAX_ATTEMPTS:
                    time.sleep(LLM_CALL_RETRY_SLEEP_S)

        self.logger.error(
            "LangChain call exhausted retries for %s: %s",
            component_name,
            last_err,
        )
        return None

    def grant_file_permissions(self, file_path: str) -> bool:
        """Grant full access permissions to file"""
        try:
            if platform.system() == 'Windows':
                # On Windows, use icacls command to grant permissions
                try:
                    # Get current username
                    username = getpass.getuser()
                    # Use icacls command to grant full control permissions
                    cmd = f'icacls "{file_path}" /grant "{username}":F'
                    subprocess.run(cmd, shell=True, check=True)
                    self.logger.info(f"Successfully granted permissions to file: {file_path}")
                    return True
                except subprocess.SubprocessError as e:
                    self.logger.error(f"Failed to grant permissions to file: {e}")
                    # Try to modify permissions using Python built-in methods
                    try:
                        os.chmod(file_path, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | \
                                        stat.S_IRGRP | stat.S_IWGRP | \
                                        stat.S_IROTH)
                        self.logger.info(f"Successfully granted permissions to file using alternative method: {file_path}")
                    except Exception as e2:
                        self.logger.error(f"Alternative permission grant method also failed: {e2}")
                    return False
            else:
                # On Linux/Mac, use chmod command
                try:
                    os.chmod(file_path, 0o755)  # rwxr-xr-x
                    self.logger.info(f"Successfully granted permissions to file: {file_path}")
                    return True
                except Exception as e:
                    self.logger.error(f"Failed to grant permissions to file: {e}")
                    return False
        except Exception as e:
            self.logger.error(f"Error during permission grant process: {e}")
            return False

    def _extract_ros_packages(self, ros_architecture: Dict[str, Any]) -> Dict[str, Any]:
        """Extract ROS package information
        
        Args:
            ros_architecture: ROS architecture data
        
        Returns:
            Dict[str, Any]: Mapping from package name to package information
        """
        packages = {}
        
        if 'ROSPackages' in ros_architecture:
            for package in ros_architecture['ROSPackages']:
                package_name = package.get('name', '')
                if package_name:
                    packages[package_name] = package
        return packages