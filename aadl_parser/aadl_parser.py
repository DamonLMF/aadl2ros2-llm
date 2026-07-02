"""aadl_parser.py

Recursively parse the AADL (Architecture Analysis & Design Language) model and convert the extracted hierarchical information into an intermediate Python data structure. 
Subsequently, this structure will be converted into XML format by the ''aadl_to_xml_converter''.

Main Function:
1. Recursively parse the package / system / implementation blocks of the specified system, and support cross-file parsing.
2. Support extraction of various AADL components such as ports, attributes, streams, attachments, connections, and sub-components.
3. The parsing process is insensitive to the case of keywords, achieved through the use of the `re.IGNORECASE` flag.
4. The parsing results are returned in a nested structure of dictionaries / lists, facilitating subsequent serialization.

eg:
    python ./aadl_parser/aadl_parser.py -i ./example/fcc -f Flight_Controller.aadl -s Flight_Controller -o ./example/fcc/Flight_Controller.xml
"""

import re
import os
import argparse
import json
from typing import Dict, List, Optional, Tuple, Any
import logging

from aadl_to_xml_converter import AADLToXMLConverter
from other_sources_scan import attach_unreferenced_c_h_as_others

class AADLParserError(Exception):
    """AADL parser custom exception class"""
    def __init__(self, message: str, component: str = "", file_path: str = "", line_number: int = 0):
        self.message = message
        self.component = component
        self.file_path = file_path
        self.line_number = line_number
        super().__init__(self.message)

class ValidationHelper:
    """Validation helper class for validating input parameters and intermediate results"""
    
    @staticmethod
    def validate_file_path(file_path: str) -> bool:
        """Validate if the file path exists and is readable"""
        if not file_path:
            return False
        return os.path.exists(file_path) and os.access(file_path, os.R_OK)
    
    @staticmethod
    def validate_string_content(content: Any, name: str = "content") -> bool:
        """Validate if the content is a valid string"""
        if content is None:
            return False
        if not isinstance(content, str):
            return False
        return True
    
    
    @staticmethod
    def safe_regex_search(pattern: str, content: str, flags: int = 0) -> Optional[re.Match]:
        """Safe regex search with error handling"""
        try:
            if not ValidationHelper.validate_string_content(content):
                return None
            return re.search(pattern, content, flags)
        except re.error as e:
            logging.getLogger(__name__).warning("Regex error for pattern %s: %s", pattern, e)
            return None
        except Exception as e:
            logging.getLogger(__name__).warning("Regex search exception: %s", e)
            return None

class AADLParser:
    """
    Recursive AADL parser.
    """

    def __init__(self):
        self.system_data = {}  # Stores parsed system data
        self.input_dir = None  # Input directory
        self.package_name  = None  # Package name where the system is implemented
        self.system_name = None  # System name
        self.errors = []
        self.warnings = []
        self.logger = logging.getLogger(__name__)
        self.validation_helper = ValidationHelper()  # Validation helper
        self.visited_components = set()  # Backward-compatible holder
        self.processing_components = set()  # Current recursion stack for cycle detection

    def _record_issue(
        self,
        issues: List[Dict[str, Any]],
        level: str,
        message: str,
        component: str = "",
        file_path: str = "",
        line_number: int = 0,
    ) -> None:
        issue = {
            'message': message,
            'component': component,
            'file_path': file_path,
            'line_number': line_number
        }
        issues.append(issue)
        log_msg = f"{level} in {file_path}:{line_number} - {component}: {message}"
        if level == "Error":
            self.logger.error(log_msg)
        else:
            self.logger.warning(log_msg)

    def add_error(self, message: str, component: str = "", file_path: str = "", line_number: int = 0):
        self._record_issue(self.errors, "Error", message, component, file_path, line_number)

    def add_warning(self, message: str, component: str = "", file_path: str = "", line_number: int = 0):
        self._record_issue(self.warnings, "Warning", message, component, file_path, line_number)

    def has_errors(self) -> bool:
        return len(self.errors) > 0

    def has_warnings(self) -> bool:
        return len(self.warnings) > 0

    def _print_issues(self, title: str, issues: List[Dict[str, Any]]) -> None:
        if not issues:
            return
        print(f"\n=== {title} ({len(issues)} {'errors' if 'error' in title.lower() else 'warnings'}) ===")
        for i, issue in enumerate(issues, 1):
            print(f"{i}. {issue['component']}: {issue['message']}")
            if issue['file_path']:
                print(f"   File: {issue['file_path']}")
            if issue['line_number']:
                print(f"   Line number: {issue['line_number']}")

    def print_summary(self):
        self._print_issues("Parsing errors", self.errors)
        self._print_issues("Parsing warnings", self.warnings)
        
    def reset_circular_reference_detection(self):
        """Reset circular reference detection state."""
        self.visited_components.clear()
        self.processing_components.clear()

    def _resolve_package_aadl_path(self, comp_package: str) -> str:
        """Resolve package AADL file path with case-insensitive fallback."""
        expected_path = os.path.join(self.input_dir, f'{comp_package}.aadl')
        if self.validation_helper.validate_file_path(expected_path):
            return expected_path

        # e.g. ROSACE::XtratuM -> rosace-xtratum.aadl (OSATE-style file naming)
        slug = str(comp_package).strip().replace("::", "-").lower()
        slug_path = os.path.join(self.input_dir, f"{slug}.aadl")
        if self.validation_helper.validate_file_path(slug_path):
            return slug_path

        if not self.input_dir or not os.path.isdir(self.input_dir):
            return expected_path

        target_name = str(comp_package).strip().lower()
        target_name_alt = target_name.replace('-', '_')
        try:
            for filename in os.listdir(self.input_dir):
                if not filename.lower().endswith('.aadl'):
                    continue
                stem = os.path.splitext(filename)[0].lower()
                stem_alt = stem.replace('-', '_')
                if (
                    stem == target_name
                    or stem_alt == target_name_alt
                    or stem == slug
                ):
                    return os.path.join(self.input_dir, filename)
        except OSError:
            return expected_path
        return expected_path

    def _resolve_source_file_path(self, relative_path: str) -> str:
        """Resolve path under input_dir for reading source; match file name case-insensitively (Linux)."""
        rel = (relative_path or "").strip().replace("\\", os.sep)
        if not rel:
            return os.path.join(self.input_dir, relative_path or "")
        direct = os.path.join(self.input_dir, rel)
        if os.path.isfile(direct):
            return direct
        if not self.input_dir or not os.path.isdir(self.input_dir):
            return direct
        parts = [p for p in rel.split(os.sep) if p]
        if not parts:
            return direct
        current = self.input_dir
        for i, part in enumerate(parts):
            is_last = i == len(parts) - 1
            candidate = os.path.join(current, part)
            if is_last and os.path.isfile(candidate):
                return candidate
            if not is_last and os.path.isdir(candidate):
                current = candidate
                continue
            try:
                match = next(
                    (n for n in os.listdir(current) if n.lower() == part.lower()),
                    None,
                )
            except OSError:
                return direct
            if match is None:
                return direct
            current = os.path.join(current, match)
        if os.path.isfile(current):
            return current
        return direct

    @staticmethod
    def _package_and_impl_from_qualified_type(comp_type: str, current_package: str) -> Tuple[str, str]:
        """Split ``Pkg::Type`` or ``Pkg::Ns::Type.impl`` into (file package slug, implementation id)."""
        if not comp_type or '::' not in comp_type:
            return current_package, comp_type
        parts = comp_type.split('::')
        if len(parts) <= 2:
            return parts[0].lower(), parts[1]
        return parts[0].lower() + '-' + parts[1].lower(), parts[2]

    @staticmethod
    def _category_and_comp_type_from_comp_def(comp_def: List[str]) -> Tuple[str, str]:
        """Derive subcomponent ``category`` and type string from the classifier tail (space-split)."""
        if len(comp_def) <= 2:
            category = comp_def[0].lower().strip()
            comp_type = comp_def[1].strip() if len(comp_def) > 1 else ''
        elif 'refined' in comp_def:
            category = comp_def[2].lower().strip()
            comp_type = comp_def[-1].strip()
        else:
            category = (comp_def[0].lower() + ' ' + comp_def[1].lower()).strip()
            comp_type = comp_def[-1].strip()
        return category, comp_type

    @staticmethod
    def _strip_subcomp_in_modes(type_str: str, subcomp_data: Dict) -> str:
        """If ``type_str`` contains ``in modes (…)``, record the first parenthesized span and return the prefix."""
        if 'modes' not in type_str or 'in modes' not in type_str:
            return type_str.strip()
        prefix, _, modes = type_str.partition('in modes')
        found = re.findall(r'\((.*?)\)', modes)
        if found:
            subcomp_data['subcomp_modes'] = found[0]
        return prefix.strip()

    def _apply_subcomp_package_impl(
        self,
        subcomp_data: Dict,
        comp_type: str,
        current_package: str,
        *,
        skip_in_modes_strip: bool = False,
    ) -> None:
        if not skip_in_modes_strip:
            comp_type = self._strip_subcomp_in_modes(comp_type, subcomp_data)
        pkg, impl = self._package_and_impl_from_qualified_type(comp_type, current_package)
        subcomp_data['package'] = pkg
        subcomp_data['implementation'] = impl

    @staticmethod
    def _new_component_data() -> Dict:
        """Create a normalized component data skeleton."""
        return {
            'name': '',
            'category': '',
            'package': '',
            'implementation': '',
            'subcomp_modes': '',
            'ports': [],
            'properties': [],
            # 'flows': [],
            'annexes': [],
            'connections': [],
            'subcomponents': [],
            # 'modes': [],
            'calls': []
        }
        
    def validate_parsing_environment(self, input_dir: str, file_path: str, system_name: str) -> bool:
        """Validate parsing environment.
        
        Args:
            input_dir: Input directory
            file_path: AADL file path
            system_name: System name
            
        Returns:
            True if environment is valid, False otherwise
        """
        # validate input directory
        if not os.path.exists(input_dir):
            self.add_error(f"Input directory does not exist: {input_dir}", "AADLParser", file_path)
            return False
            
        if not os.path.isdir(input_dir):
            self.add_error(f"Input path is not a directory: {input_dir}", "AADLParser", file_path)
            return False
            
        # validate file path
        if not self.validation_helper.validate_file_path(file_path):
            self.add_error(f"File does not exist or cannot be read: {file_path}", "AADLParser", file_path)
            return False
            
        # validate system name
        if not system_name or not isinstance(system_name, str):
            self.add_error("System name is invalid", "AADLParser", file_path)
            return False
            
        return True
        
    def validate_parsed_data(self, system_data: Dict) -> bool:
        """Validate parsed data.
        
        Args:
            system_data: Parsed system data
            
        Returns:
            True if data is valid, False otherwise
        """
        if not system_data:
            self.add_error("Parsed data is empty", "AADLParser", getattr(self, 'file_path', ''))
            return False
            
        required_fields = ['name', 'package', 'category', 'implementation']
        for field in required_fields:
            if field not in system_data:
                self.add_error(f"Required field {field} is missing", "AADLParser", getattr(self, 'file_path', ''))
                return False
                
        return True
        
    def detect_circular_references(self, component_name: str) -> bool:
        """Detect circular references.
        
        Args:
            component_name: Component name to check
            
        Returns:
            True if circular reference is detected, False otherwise
        """
        if component_name in self.processing_components:
            self.add_warning(f"Circular reference detected: {component_name}", component_name, getattr(self, 'file_path', ''))
            return True
        return False
        
    def validate_final_result(self, system_data: Dict) -> bool:
        """Validate final parsed result.
        
        Args:
            system_data: Final parsed system data
            
        Returns:
            True if result is valid, False otherwise
        """
        if not self.validate_parsed_data(system_data):
            return False
            
        # port validation
        if 'ports' in system_data:
            for port in system_data['ports']:
                if not isinstance(port, dict) or 'name' not in port:
                    self.add_warning("Port data format is incorrect", system_data.get('name', 'Unknown'), self.file_path)
                    
        # property validation
        if 'properties' in system_data:
            for prop in system_data['properties']:
                if not isinstance(prop, dict) or 'name' not in prop:
                    self.add_warning("Property data format is incorrect", system_data.get('name', 'Unknown'), self.file_path)
        return True

    def _parse_package_file(self, input_dir: str, file_path: str, system_name: str) -> Dict:
        """Parse specified AADL system and its subcomponents.
        
        Args:
            input_dir: Input directory
            file_path: AADL file path
            system_name: System name
            
        Returns:
            Parsed system data if successful, empty dict otherwise
        """
        # reset circular reference detection state
        self.reset_circular_reference_detection()
        
        # validate parsing environment
        if not self.validate_parsing_environment(input_dir, file_path, system_name):
            return {}
            
        self.input_dir = input_dir # input dir
        self.system_name = system_name # system name
        self.file_path = file_path  # file path
        
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Remove the comments
        content = self._remove_comments(content)
        
        # parse package name (AADL keywords are case-insensitive)
        package_match = self.validation_helper.safe_regex_search(
            r'package\s+([\w:.]+)', content, re.IGNORECASE
        )
        if not package_match:
            self.add_warning(f"Package name not found", "AADLParser", file_path)
            return {}
        package_name = package_match.group(1)
        self.package_name = package_name  # system package name
        
        # find system definition
        system_match = self.validation_helper.safe_regex_search(
            rf'(?:system|abstract|subprogram|thread)\s*{system_name}\s*(.*?)\s*{system_name};', 
            content, re.DOTALL | re.IGNORECASE)

        if not system_match:
            self.add_error(f"System definition not found: {system_name}", "AADLParser", file_path)
            return {}
        
        
        system_content = system_match.group(1)
        # parse implementation declarations for the target system name
        impl_names = re.findall(
            rf'\b(system|abstract|process|thread|subprogram)\s+implementation\s+{re.escape(system_name)}\.(\w+)\b',
            content,
            re.IGNORECASE
        )
        if not impl_names:
            print(f"Warning: No implementation found in {file_path}")
            return {}
        systems = []
        for category, impl_name in impl_names:
            system_data = self._new_component_data()
            system_data.update({
                'name': system_name,
                'package': package_name,
                'category': category.lower().strip(),
                'implementation': f"{system_name}.{impl_name}",
                'ports': self._parse_ports(system_content, package_name),
                'properties': self._parse_properties(system_content),
                # 'flows': self._parse_flows(system_content),
                'annexes': self._parse_annexes(system_content),
            })
            system_data = self._parse_impl_content(system_data, content)
            systems.append(system_data)
        attach_unreferenced_c_h_as_others(input_dir, systems)
        return systems

    def _parse_subcomponents(self, content: str, system_data: Dict) -> List[Dict]:
        """Parse subcomponents and their all features""" 
        current_package = system_data['package']
        subcomponents = []
        # print(f"44444444444Parsing subcomponents...{content}")
        subcomp_match = re.search(r'subcomponents\s*(.*?)(?:connections|;\s*properties|;\s*end)', content, re.DOTALL | re.IGNORECASE | re.MULTILINE)
        if subcomp_match is None:
            return subcomponents
        
        subcomp_content = subcomp_match.group(1)
        if not subcomp_content.endswith(';'):
            subcomp_content += ';'
        if '{' in subcomp_content: # he properties of the sub-components are defined within {.
            subcomp_pattern = r'\s*(\w+)\s*:\s*(.*?)\s*(?:\s*\{(.*?)\})?;' # Matching thread component
            subcomp_matches = re.finditer(subcomp_pattern, subcomp_content, re.DOTALL | re.IGNORECASE)
            for subcomp_match in subcomp_matches:
                if subcomp_match.group(0) is None:
                    continue
                subcomp_data = self._new_component_data()
                subcomp_data['name'] = subcomp_match.group(1)
                comp_def = subcomp_match.group(2).split(' ')
                subcomp_data['category'], comp_type = self._category_and_comp_type_from_comp_def(comp_def)
                self._apply_subcomp_package_impl(subcomp_data, comp_type, current_package)
                properties_line = subcomp_match.group(3)
                if properties_line:
                    properties_line = properties_line.split(';')
                    for prop in properties_line:
                        prop = prop.strip()
                        if '=>' not in prop:
                            continue
                        name, value = prop.split('=>', 1)
                        name = name.strip()
                        value = value.strip()
                        if '::' in name:
                            package, prop_name = name.split('::', 1)
                            subcomp_data['properties'].append({
                                'name': prop_name.strip().lower(),
                                'package': package.strip(),
                                'value': value
                            })
                        else:
                            subcomp_data['properties'].append({
                                'name': name.lower(),
                                'package': 'default',
                                'value': value
                            })
                subcomponents.append(subcomp_data)              
        else:
            subcomp_lines = [line.strip() for line in subcomp_content.split(';') if line.strip()]
            for line in subcomp_lines:
                subcomp_data = self._new_component_data()
                line = self._strip_subcomp_in_modes(line, subcomp_data)
                name, comp_def = line.split(':', 1)
                subcomp_data['name'] = name.strip()
                comp_def = comp_def.strip().split(' ')
                subcomp_data['category'], comp_type = self._category_and_comp_type_from_comp_def(comp_def)
                self._apply_subcomp_package_impl(subcomp_data, comp_type, current_package, skip_in_modes_strip=True)
                subcomponents.append(subcomp_data)
        for subcomp in subcomponents:
            # Detect circular references
            if self.detect_circular_references(subcomp['implementation']):
                print(f"Due to circular references, skipping component: {subcomp['implementation']}")
                continue

            current_impl = subcomp.get('implementation', '')
            if current_impl:
                self.processing_components.add(current_impl)
            try:
                subcomp, subcomp_content = self._parse_source_content(subcomp)
            except Exception as e:
                self.add_error(
                    f"An error occurred while parsing the definition of the sub-component： {str(e)}",
                    subcomp['name'],
                    self.file_path,
                )
            else:
                try:
                    if subcomp_content and subcomp_content.strip():
                        parsed_subcomp = self._parse_impl_content(subcomp, subcomp_content)
                        if parsed_subcomp is not None:
                            subcomp = parsed_subcomp
                    else:
                        self.add_warning(
                            f"Component implementation content is empty",
                            subcomp['implementation'],
                            self.file_path,
                        )
                except Exception as e:
                    self.add_error(
                        f"An error occurred while parsing the implementation of the component： {str(e)}",
                        subcomp['implementation'],
                        self.file_path,
                    )
            finally:
                if current_impl:
                    self.processing_components.discard(current_impl)
        return subcomponents 

    def _parse_source_content(self, subcomp: Dict) -> Tuple[Dict, Optional[str]]:
        """Parse the source content of the sub-component"""
            
        comp_package = subcomp['package']
        comp_impl = subcomp['implementation']
        category = subcomp['category'].lower().strip()
        subcomp_path = self._resolve_package_aadl_path(comp_package)
        
        # validate file path
        if not self.validation_helper.validate_file_path(subcomp_path):
            self.add_warning(f"Component file does not exist: {subcomp_path}", subcomp['name'], self.file_path)
            return subcomp, ""
            
        try:
            with open(subcomp_path, 'r', encoding='utf-8') as f:
                subcomp_content = f.read()
                subcomp_content = self._remove_comments(subcomp_content)
        except FileNotFoundError:
            self.add_error(f"Component file does not exist: {subcomp_path}", subcomp['name'], self.file_path)
            return subcomp, ""
        except Exception as e:
            self.add_error(f"An unknown error occurred while reading the component file: {str(e)}", subcomp['name'], self.file_path)
            return subcomp, ""
            
        try:
            subcomp_source_name = comp_impl.split(".")[0] if "." in comp_impl else comp_impl
            # Search for component definitions
            # Require line-start match so subcomponent lines like ``vrp: data int {...}``
            # do not capture text up to the next ``end int;`` type definition.
            comp_def_pattern = (
                rf'(?:^|\n)\s*{re.escape(category)}\s+{re.escape(subcomp_source_name)}\s+'
                rf'(.*?)end\s*{re.escape(subcomp_source_name)};'
            )
            comp_def_match = re.search(
                comp_def_pattern, subcomp_content, re.DOTALL | re.IGNORECASE | re.MULTILINE
            )
            if comp_def_match and comp_def_match.group(1) is not None:
                comp_def_content = comp_def_match.group(1) + 'end'
                # print(f"Component {subcomp_source_name} is defined: {comp_def_content}")
                subcomp['ports'].extend(self._parse_ports(comp_def_content, subcomp['package']))
                # Parse properties in the component definition
                subcomp['properties'].extend(self._parse_properties(comp_def_content))
                subcomp['properties'] = self._dedupe_properties_by_name(subcomp['properties'])
                # if 'flows' in subcomp:
                #     subcomp['flows'].extend(self._parse_flows(comp_def_content))
                if 'annexes' in subcomp:
                    subcomp['annexes'].extend(self._parse_annexes(comp_def_content))
            else:
                self.add_warning(f"Component definition not found: {subcomp_source_name}", subcomp['name'], subcomp_path)
        except Exception as e:
            self.add_error(f"An error occurred while processing the component file: {str(e)}", subcomp['name'], subcomp_path)
        return subcomp, subcomp_content

    def _parse_impl_content(self, system_data: Dict, content: str) -> Dict:
        """Parse the implementation content of the system"""
        # Validate input parameters
        if not self.validation_helper.validate_string_content(content):
            self.add_error("Implementation content is empty or invalid", system_data.get('implementation', 'Unknown'), self.file_path)
            return system_data
            
        comp_package = system_data['package']
        impl_name = system_data['implementation']
        impl_match = self.validation_helper.safe_regex_search(
            rf"{system_data['category']}\s*implementation\s*{impl_name}\s*(.*?)\s*end\s*{impl_name};",
            content,
            re.DOTALL | re.IGNORECASE
        )

        # Fallback for cases where subcomponents reference type name (e.g., Foo) and
        # properties are declared in Foo.impl.
        if (impl_match is None or impl_match.group(1) is None) and '.' not in impl_name:
            alt_impl_name = f"{impl_name}.impl"
            alt_match = self.validation_helper.safe_regex_search(
                rf"{system_data['category']}\s*implementation\s*{alt_impl_name}\s*(.*?)\s*end\s*{alt_impl_name};",
                content,
                re.DOTALL | re.IGNORECASE
            )
            if alt_match is not None and alt_match.group(1) is not None:
                impl_match = alt_match
                system_data['implementation'] = alt_impl_name
        
        if impl_match is not None and impl_match.group(1) is not None:
            impl_content = impl_match.group(1) + '\n' + 'end'
            
            # parse implementation-specific properties, connections, etc.
            # Requirement: do not parse calls in process components.
            if str(system_data.get('category', '')).strip().lower() != 'process':
                impl_calls = self._parse_calls(impl_content, comp_package)
                if impl_calls:
                    system_data['calls'].extend(impl_calls)

            impl_properties = self._parse_properties(impl_content)
            if impl_properties:
                system_data['properties'].extend(impl_properties)
                system_data['properties'] = self._remove_duplicate_dicts(list(system_data['properties']))
            
            # impl_flows = self._parse_flows(impl_content)
            # if impl_flows:
            #     system_data['flows'].extend(impl_flows)
            impl_annexes = self._parse_annexes(impl_content)
            if impl_annexes:
                system_data['annexes'].extend(impl_annexes)
            
            # impl_modes = self._parse_modes(impl_content)
            # if impl_modes:
            #     system_data['modes'].extend(impl_modes)

            # recursively parse subcomponents
            subcomponents = self._parse_subcomponents(impl_content, system_data)
            if subcomponents:
                system_data['subcomponents'].extend(subcomponents)

            impl_connections = self._parse_connections(system_data, impl_content)
            if impl_connections:
                system_data['connections'].extend(impl_connections)
            return system_data
        return system_data

    def _parse_connections(self, system_data: Dict, content: str) -> List[Dict]:
        """Parse connections in the implementation of the system"""
        component_name = system_data['name']
        category = system_data['category'].lower()
        subcomponents = system_data['subcomponents']
        connections = []
        try:
            # print(f"Parsing connection content: {content}")

            conn_match = re.search(r'connections\s*(.*?)(?=\nend|properties|flows|subcomponents|modes\n|annex)', content, re.DOTALL | re.IGNORECASE)
            if not conn_match:
                return connections
            conn_lines = [line.strip() for line in conn_match.group(1).split(';') if line.strip()]
            
            for line in conn_lines:
                try:
                    if ':' not in line:
                        continue
                    conn_def = re.search(r'(\w+)\s*:\s*((?:bus\s+access|port|data\s+access|parameter)?)\s+([^\s]+)\s*(?:->|<->)\s*([^\s]+)(?:\s+in\s+modes\s*\(([^\)]+)\))?', line, re.DOTALL | re.IGNORECASE)
                    if not conn_def:
                        continue
                    name = conn_def.group(1).strip().lower()
                    conn_type = conn_def.group(2).strip() + ' connection'
                    source = conn_def.group(3).strip()
                    destination = conn_def.group(4).strip()
                    
                    if 'port' in conn_type.lower() or 'parameter' in conn_type.lower():
                        # Handle source and destination components and ports
                        if "." not in source:
                            source = component_name + '.' + source
                            conn_type = 'process2thread ' + conn_type
                            name = '/' + source.replace('.', '/')
                        elif "." not in destination:
                            destination = component_name + '.' + destination
                            conn_type = 'thread2process ' + conn_type
                            name = '/' + destination.replace('.', '/')
                        elif category == 'process':
                            conn_type = 'same-level thread ' + conn_type
                            name = '/' + component_name + '_' + name
                        elif category == 'system':
                            src_is_process = any(
                                sc['name'] in source and sc['category'].strip().lower() == 'process'
                                for sc in subcomponents
                            )
                            dst_is_process = any(
                                sc['name'] in destination and sc['category'].strip().lower() == 'process'
                                for sc in subcomponents
                            )
                            if src_is_process and dst_is_process:
                                conn_type = 'process2process ' + conn_type
                                name = '/' + source.replace('.', '/')
                            elif src_is_process:
                                conn_type = 'process2device ' + conn_type
                                name = '/' + source.replace('.', '/')
                            elif dst_is_process:
                                conn_type = 'device2process ' + conn_type
                                name = '/' + destination.replace('.', '/')

                    if 'data' in conn_type:
                        conn_type = 'data access connection'
                    elif 'bus' in conn_type:
                        conn_type = 'bus access connection'

                    # Build connection information
                    connection = {
                        'name': name.lower(),
                        'type': conn_type,
                        'source': source.lower(),
                        'destination': destination.lower(),
                        # 'modes': ""
                    }
                    
                    # Handle mode information
                    # if len(conn_def.groups()) > 4 and conn_def.group(5):
                    #     modes = conn_def.group(5).strip()
                    #     connection['modes'] = modes
                    connections.append(connection)
                except Exception as e:
                    print(f"Error parsing connection details in line '{line}': {str(e)}")
                    
        except Exception as e:
            print(f"Error parsing connections section in content '{content}': {str(e)}")
            
        return connections

    def _parse_modes(self, content: str) -> List[Dict]:
        """Parse modes in the implementation of the system"""
        modes = []
        try:
            mode_match = re.search(r';\s*modes\s*(.*?)\s*(?=end|flows|annex|properties)', content, re.DOTALL | re.IGNORECASE)
            if mode_match:
                mode_content = mode_match.group(1)
                mode_lines = mode_content.strip().split('\n')
                for line in mode_lines:
                    line = line.strip()
                    if ':' in line:
                        mode_name, mode_value = line.split(':')
                        mode = {
                            'name': mode_name.strip(),
                            'type': mode_value.replace(';', '').strip()
                        }
                        modes.append(mode)
                    else:
                        modes.append({
                            'transition': line.strip()
                        })
        except Exception as e:
            print(f"Error parsing modes section: {str(e)}")
        return modes

    def _parse_calls(self, impl_content: str, current_package: str) -> List[Dict]:
        """Parse calls in the implementation of the system"""
        calls = []
        calls_match = re.search(r'calls\s*(.*?)\s*(?=\nend|connections|modes|flows|annex|properties)', impl_content, re.DOTALL | re.IGNORECASE)
        if calls_match:
            calls_content = calls_match.group(1)
            # print(f"calls_content: {calls_content}")
            calls_content = re.findall(r'(.*?)\s*:\s*{\s*(.*?)\s*:\s*subprogram\s*(.*?);\s*};', calls_content, re.DOTALL | re.IGNORECASE)
            #  Parse calls section
            for call in calls_content:
                #  Define call dictionary
                s_call = {
                    'call_name': '',
                    'subprogram_name': '',
                    'subprogram_impl': '',
                    'subprogram_package': '',
                    'subprogram_port': [],
                    'subprogram_properties': []
                }
                s_call['call_name'] = call[0].strip()
                s_call['subprogram_name'] = call[1].strip()
                subprogram_impl = call[2].strip()
                if '::' in subprogram_impl:
                    comp_type = subprogram_impl.split('::')
                    if len(comp_type) <= 2:
                        s_call['subprogram_package'] = comp_type[0]
                        s_call['subprogram_impl'] = comp_type[1]
                    else:
                        s_call['subprogram_package'] = comp_type[0] + '-' + comp_type[1]
                        s_call['subprogram_impl'] = comp_type[2]
                else:
                    s_call['subprogram_package'] = current_package
                    s_call['subprogram_impl'] = subprogram_impl
                # Resolve package file path with case-insensitive fallback.
                subprogram_package_path = self._resolve_package_aadl_path(s_call['subprogram_package'])
                if self.validation_helper.validate_file_path(subprogram_package_path):
                    with open(subprogram_package_path, "r", encoding='utf-8') as f:
                        content = self._remove_comments(f.read())
                        # Match declaration blocks only; full-file `subprogram T` hits false positives in CALLS.
                        sub_name = re.escape(s_call["subprogram_impl"])
                        subprogram_match = re.search(
                            rf'^\s*subprogram\s+{sub_name}\b(?!\s*\.)\s*(.*?)(?=^\s*end\s+{sub_name}\s*;)\s*^\s*end\s+{sub_name}\s*;',
                            content,
                            re.DOTALL | re.IGNORECASE | re.MULTILINE
                        )
                        if subprogram_match:
                            subprogram_content = subprogram_match.group(1) + '\nend'
                            s_call['subprogram_port'] = self._parse_ports(subprogram_content, s_call['subprogram_package'])
                            s_call['subprogram_properties'] = self._parse_properties(subprogram_content)
                        # AADL often uses subprogram implementation Name.impl ... end Name.impl;
                        # as well as subprogram implementation Name ... end Name;
                        _base = s_call['subprogram_impl']
                        for impl_id in (_base, f'{_base}.impl'):
                            impl_s = re.escape(impl_id)
                            subprogram_impl_match = re.search(
                                rf'^\s*subprogram\s*implementation\s+{impl_s}\b\s*(.*?)(?=^\s*end\s+{impl_s}\b\s*;)\s*^\s*end\s+{impl_s}\b\s*;',
                                content,
                                re.DOTALL | re.IGNORECASE | re.MULTILINE
                            )
                            if subprogram_impl_match:
                                subprogram_impl_content = subprogram_impl_match.group(1) + '\nend'
                                s_call['subprogram_properties'].extend(self._parse_properties(subprogram_impl_content))
                                break
                calls.append(s_call)
        return calls

    def _parse_flows(self, content: str) -> List[Dict]:
        flows = []
        try:
            if 'end to end' in content:
                flow_match = re.search(r'flows\s*(.*?)(?=properties)', content, re.DOTALL | re.IGNORECASE)
            else:
                flow_match = re.search(r'flows\s*(.*?)(?=properties|end)', content, re.DOTALL | re.IGNORECASE)
            if not flow_match:
                return flows
                
            flow_content = flow_match.group(1)
            if '{' in flow_content:
                flow_lines = [line.strip() for line in flow_content.split(';}') if line.strip()]  # FIXME: brittle split
            else:
                flow_lines = [line.strip() for line in flow_content.split(';') if line.strip()]
            for line in flow_lines:
                try:
                    if ':' not in line:
                        continue
                    flow = {
                        'name': '',
                        'type': '',
                        'source': '',
                        'destination': '',
                        'path': '',
                        'properties': []
                    }
                    if '{' in line:
                        flow_content , flow_properties = line.split('{',1)
                        if ';' in flow_properties:
                            flow_properties = flow_properties.split(';')
                            for flow_properties in flow_properties:
                                flow_properties_name, flow_properties_value = flow_properties.split('=>')
                                flow['properties'].append({
                                    'property_name': flow_properties_name.strip().lower(),
                                    'property_value': flow_properties_value.strip()
                                })
                        else:
                            flow_properties_name, flow_properties_value = flow_properties.split('=>')
                            flow['properties'].append({
                                    'property_name': flow_properties_name.strip().lower(),
                                    'property_value': flow_properties_value.strip()
                                })
                    else:
                        flow_content = line
                    flow_re = r'(.*?):\s*(flow source|flow sink|flow path|end to end flow)\s*(.*)'
                    flow_info = re.search(flow_re, flow_content, re.DOTALL | re.IGNORECASE)
                    if flow_info is None:
                        continue
                    flow['name'] = flow_info.group(1).strip()
                    flow['type'] = flow_info.group(2).strip()
                    flow_path = flow_info.group(3).strip()
                    if flow['type'] == 'flow source':
                        flow['source'] = flow_path.strip()
                    elif flow['type'] == 'flow sink':
                        flow['destination'] = flow_path.strip()
                    else:
                        path_parts = [p.strip() for p in flow_path.split('->')]
                        flow['source'] = path_parts[0]
                        flow['destination'] = path_parts[-1]
                        flow['path'] = ','.join(path_parts)
                    #  Add flow to list
                    flows.append(flow)
                except Exception as e:
                    print(f"Error parsing flow line '{line}': {str(e)}")
                    continue
        except Exception as e:
            print(f"Error parsing flows section: {str(e)}")
            
        return flows    

    def _parse_properties(self, content: str) -> List[Dict]:
        """Parse properties in the implementation of the system"""
        properties = []
        seen: set = set()
        _pk = lambda p, n: f"{(p or '').strip().lower()}::{(n or '').strip().lower()}"
        try:
            prop_match = re.search(r'properties\s*\n(.*?)(?=^\s*(?:end|connections|flows|subcomponents|modes|annex))', content, re.DOTALL |re.IGNORECASE| re.MULTILINE)
            if not prop_match:
                return properties
            # print(f"Property line: {prop_match.group(1)}")
            prop_lines = prop_match.group(1).split(';')
            for line in prop_lines:
                try:
                    line = line.strip()
                    if not line or '=>' not in line:
                        continue
                        
                    name, value = line.split('=>', 1)
                    name = name.strip()
                    if '(' in value:
                        value = value.strip().replace('(', '').replace(')', '')
                    if '"' in value:
                        value = value.strip().replace('"', '')
                    
                    if '::' in name:
                        package, prop_name = name.split('::', 1)
                        pkg, pn = package.strip(), prop_name.strip()
                        k = _pk(pkg, pn)
                        if k in seen:
                            continue
                        seen.add(k)
                        properties.append({
                            'name': pn.lower(),
                            'package': pkg,
                            'value': value.strip()
                        })
                    elif name.lower() == 'source_text':
                        # source_text
                        rel_name = value.strip()
                        k = _pk(rel_name, name)
                        if k in seen:
                            continue
                        seen.add(k)
                        properties.append({
                            'name': name.lower(),
                            'package': rel_name,
                            'value': "",
                        })
                        try:
                            source_code_path = self._resolve_source_file_path(rel_name)
                            with open(source_code_path, 'r', encoding='utf-8') as f:
                                content = f.read()
                                source_code = ' '.join(content.split())
                            # Modify the last added element, rather than trying to use a string to index the list
                            properties[-1]['value'] = source_code
                        except Exception as e:
                            print(f'Error reading source_text code file: {e}')

                    elif name.lower() == 'source_name':
                        #  Add source_name property, and add source_text property
                        k = _pk('default', name)
                        if k in seen:
                            continue
                        seen.add(k)
                        properties.append({
                            'name': name.lower(),
                            'package': 'default',
                            'value': value.strip()
                        })
                        
                        # Generate the Source_text value based on the Source_name value.
                        # For example: If the Source_name value is a.b, then the Source_text is a.adb
                        if '.' in value:
                            base_name = value.split('.')[0]
                            source_text_value = f'{base_name}.adb'
                            
                            # Add source_text property
                            try:
                                st_pkg = source_text_value.strip()
                                k2 = _pk(st_pkg, 'source_text')
                                if k2 in seen:
                                    continue
                                source_code_path = self._resolve_source_file_path(st_pkg)
                                with open(source_code_path, 'r', encoding='utf-8') as f:
                                    content = f.read()
                                    source_code = ' '.join(content.split())
                                seen.add(k2)
                                properties.append({
                                    'name': 'source_text',
                                    'package': st_pkg,
                                    'value': source_code,
                                })
                            except Exception as e:
                                print(f'Error reading auto-generated source_text file: {e}')

                    else:
                        k = _pk('default', name)
                        if k in seen:
                            continue
                        seen.add(k)
                        properties.append({
                            'name': name.lower(),
                            'package': 'default',
                            'value': value.strip()
                        })
                
                except Exception as e:
                    print(f"Error parsing property line '{line}': {str(e)}")
                    continue
        except Exception as e:
            print(f"Error parsing properties section: {str(e)}")
        return properties

    def _parse_annexes(self, content: str) -> List[Dict]:
        """Parse annexes in the implementation of the system"""
        annexes = []
        try:
            annexes_match = re.finditer(r'annex\s*(.*?)\s*{\*\*(.*?)\*\*};', content, re.DOTALL | re.IGNORECASE)
            if not annexes_match:
                return annexes  
            for match in annexes_match:
                try:
                    annex_type = match.group(1)  # get annex type(name)
                    annex_body = match.group(2).strip()  # get annex content，and remove the beginning and end white space
                    # Remove line breaks and extra whitespace
                    annex_body = ' '.join(annex_body.split())
                            
                    annexes.append({
                        'name': annex_type,
                        # 'type': annex_type,
                        'body': annex_body,
                    })
                except Exception as e:
                    print(f"Error parsing annex: {str(e)}")
                    continue
        except Exception as e:
            print(f"Error parsing annexes section: {str(e)}")
        return annexes

    def _parse_ports(self, content: str, current_package: str) -> List[Dict]:
        """Parse ports in the implementation of the system"""
        ports = []
        # Allow indented "end" (e.g. "\n  end") and EOF as section boundaries.
        port_match = re.search(
            r'features(.*?)(?=properties|flows|annex|\n\s*end|\Z)',
            content,
            re.DOTALL | re.IGNORECASE
        )
        if not port_match:
            return ports  # No port definition found, return empty list
        
        port_content = port_match.group(1)
        port_lines = [line.strip() for line in port_content.split(';') if line.strip()]
        
        for line in port_lines:
        # Only skip processing when a line does not contain 'port', 'access', or 'parameter'
            if 'port' not in line.lower() and 'access' not in line.lower() and 'parameter' not in line.lower():
                # print(f"line: {line}")
                continue
            name, port_def = line.split(':', 1)
            name = name.strip()
            port_def = port_def.strip()

            # Determine port type (data port, event port, event data port, etc.)
            port_kind = None
            data_type = None
            initial_value = None
            # Build regex to capture data type
            # Match pattern: [direction] [bus access|event|data|event data] port [data type]
            # First check if it's a bus access port
            if 'access' in port_def.lower():
                port_type_match = re.search(
                    r'(requires|provides)\s*((?:bus\s*access)?(?:data\s*access)?)\s*(?:\s*([^\s;]+))?', 
                    port_def,
                    re.DOTALL | re.IGNORECASE
                )
                if port_type_match:
                    port_kind = 'bus access'
            elif 'parameter' in port_def.lower():
                if "Initial_Value" in port_def:
                    port_def, properties_content = port_def.split("{", 1)
                    properties_value = properties_content.split("=>", 1)[-1].strip()
                    initial_value= properties_value.replace('"', '').replace('(', '').replace(')', '')
                port_type_match = re.search(
                    r'(in|out|in\s*out)\s*(parameter)(?:\s*([^\s;]+))?', 
                    port_def,
                    re.DOTALL | re.IGNORECASE
                )
                if port_type_match:
                    port_kind = 'parameter'
            else:
                # Handle other types of ports
                port_type_match = re.search(
                    r'(in|out|in\s*out)\s*((?:event\s*data)?(?:event)?(?:data)?)\s*port(?:\s*([^\s;]+))?', 
                    port_def,
                    re.DOTALL | re.IGNORECASE
                )
            
            if port_type_match:
                # print('port_type_match:', port_type_match.group(0))
                direction = port_type_match.group(1).strip()
                port_kind = port_type_match.group(2).strip()

                # If found, parse data type
                if port_type_match.group(3):
                    data_type = port_type_match.group(3).strip()
                    # Handle package-qualified types using :: symbol
                    if '::' in data_type:
                        package_name, type_name = data_type.split('::', 1)
                    else:
                        package_name, type_name = current_package, data_type

                    data_type = {
                        'name': type_name.strip(),
                        'package': package_name.strip(),
                    }
                    # Get detailed data component information
                    data_component = self._parse_data_component(type_name, package_name)
                    if data_component:
                        # If subcomponents exist, add them
                        if data_component.get('subcomponents'):
                            data_type['subcomponents'] = data_component.get('subcomponents', [])
                        # Store top-level data component properties
                        if data_component.get('properties'):
                            data_type['properties'] = data_component.get('properties', [])
                # Build port data
                port_data = {
                    'name': name.lower(),
                    'direction': direction,
                    'port_kind': port_kind,
                    'initial_value': initial_value if initial_value else '',
                    'data_type': data_type if data_type else None,
                }
                ports.append(port_data)
        return ports

    def _parse_data_component(self, type_name: str, package_name: str) -> Dict:
        """Parse the data component definitions from the AADL file """
        # Construct the file path of AADL
        data_file_path = os.path.join(self.input_dir, f"{package_name}.aadl")
        if not os.path.exists(data_file_path):
            data_file_path = os.path.join(self.input_dir, f"{package_name.lower()}.aadl")
            if not os.path.exists(data_file_path):
                return None
        try:
            with open(data_file_path, 'r', encoding='utf-8') as f:
                file_content = self._remove_comments(f.read())
                
                # create data component structure
                data_component = {
                    'subcomponents': [],
                    'properties': []
                }
                data_pattern = rf'data\s*{type_name}\s*(.*?)\s*{type_name};'
                data_match = re.search(data_pattern, file_content, re.DOTALL | re.IGNORECASE)
                if data_match:
                    data_content = data_match.group(1).strip()
                    props = self._parse_properties(data_content)
                    if props:
                        data_component['properties'].extend(props)

                # Utilize simple and reliable pattern matching
                # Match patterns implemented by 'bus' or 'data', supporting the format 'end TypeName;'
                impl_pattern = rf'(?:bus|data)\s*implementation\s*{type_name}\s*(.*?)\s*{type_name};'
                impl_match = re.search(impl_pattern, file_content, re.DOTALL | re.IGNORECASE)
                if impl_match:
                    impl_content = impl_match.group(1).strip()  # extract implementation content block, in the first capture group of the regular expression
                    # print(f"9999999999999data implementation mathces: {impl_content}")
                    
                    subcomp_match = re.search(r'subcomponents\s*(.*?)(?:properties|end)', impl_content, re.DOTALL | re.IGNORECASE)
                    if subcomp_match:
                        subcomps = subcomp_match.group(1).strip()
                        for line in subcomps.split(';'):
                            line = line.strip()
                            if not line:
                                continue
                            subcomp_name, subcomp_def = line.split(':', 1)
                            # More carefully extract the types of subcomponents
                            subcomp_type = subcomp_def.replace('data', '').strip()
                            # Only add subcomponents that contain both name and type
                            data_component['subcomponents'].append({
                                'name': subcomp_name.strip(),
                                'type': subcomp_type.strip()
                            })
                    # Extract properties from the implementation block
                    props = self._parse_properties(impl_content)
                    if props:
                        data_component['properties'].extend(props)
                
                return data_component
                
        except Exception as e:
            print(f"Error parsing data component: {package_name}::{type_name}: {str(e)}")
            return None
    
    def _parse_individual_properties(self, line: str) -> Dict:
        """Parse a single property definition, format: package::property_name => value;"""
        properties = {}
        if '{' in line:
            line = line.split('{', 1)[1]  # Only parse the content after {
        if '=>' in line:
            name, value = line.split('=>', 1)
            if '::' in name:
                package, prop_name = name.split('::', 1)
                properties['package'] = package.strip()
                properties['name'] = prop_name.strip().lower()
            else:
                properties['name'] = name.strip().lower()
            #  If the value contains numbers, only keep the number part (supporting integers or decimals)
            num_matches = re.findall(r'[\d.]+', value)
            if num_matches:
                # If there are multiple numbers (e.g., range 1 .. 10), join them with spaces
                properties['value'] = ' '.join(num_matches)
            else:
                properties['value'] = value.strip()
        return properties
    
    def _parse_properties_block(self, content: str) -> str:
        """
            Extract property blocks that span multiple lines
            Example:
            properties_names {
                package::prop1 => 1;
                package::prop2 => 2;
            }
        """
        prop_block = ''
        brace_count = 0
        for line in content.splitlines():
            if '{' in line:
                brace_count += 1
            if '}' in line:
                brace_count -= 1
            if brace_count > 0 or ('{' in line and '}' in line):
                prop_block += line.strip() + ' '
            if brace_count == 0 and prop_block:
                break
        return prop_block.strip()
    
    def _remove_comments(self, content: str) -> str:
        """Remove comments from AADL files"""
        # Split content into lines
        lines = content.split('\n')
        # Remove comments after '--' on each line
        cleaned_lines = []
        for line in lines:
            comment_pos = line.find('--')
            if comment_pos != -1:
                # Keep only the content before the comment symbol
                line = line[:comment_pos]
            if line.strip():  # only keep non-empty lines
                cleaned_lines.append(line)
        return '\n'.join(cleaned_lines)  # return the content after removing comments

    def _remove_duplicate_dicts(self, lst: List[Dict]) -> List[Dict]:
        """
        Remove duplicate dictionary items from a list, preserving order, and key-value pair order does not matter
        """
        seen = set()
        unique = []
        for d in lst:
            key = frozenset(d.items())  # dict -> hashable unique identifier
            if key not in seen:
                seen.add(key)
                unique.append(d)
        return unique

    def _dedupe_properties_by_name(self, lst: List[Dict]) -> List[Dict]:
        """Keep first occurrence per ``name`` (case-insensitive)."""
        seen = set()
        out: List[Dict] = []
        for d in lst or []:
            if not isinstance(d, dict):
                continue
            k = (d.get("name") or "").strip().lower()
            if k:
                if k in seen:
                    continue
                seen.add(k)
            out.append(d)
        return out


def main():
    parser = argparse.ArgumentParser(description="Convert AADL files to XML.")
    parser.add_argument("-i", "--input_dir", required=True, help="Directory containing AADL files to be converted")
    parser.add_argument("-o", "--output", required=True, help="Output directory or XML file path")
    parser.add_argument("-f", "--file_name", required=True, help="AADL model file to be converted")
    parser.add_argument("-s", "--system", required=True, help="Top-level AADL system name to parse")
    args = parser.parse_args()

    aadl_parser = AADLParser()
    # converter = AADLToXMLConverter()
    
    try:
        # Parse the specified AADL system implementation
        file_path = os.path.join(args.input_dir, args.file_name) # Get the file path
        complete_system = aadl_parser._parse_package_file(args.input_dir, file_path, args.system)
        
        # Check the parsing results
        if not complete_system:
            print("Error: Parsing failed, no system data generated")
            aadl_parser.print_summary()
            return
            
        # Validate the final parsing results
        if not aadl_parser.validate_final_result(complete_system):
            print("Warning: Parsing results validation failed")
            
        # Handle the output path, supporting both file paths and directory paths
        output_arg = args.output
        if output_arg.lower().endswith('.xml'):
            output_file = output_arg
            output_dir = os.path.dirname(output_file) or '.'
        else:
            output_dir = output_arg
            output_file = os.path.join(output_dir, f'{args.system}.xml')

        # Ensure output directory exists before writing any artifacts
        os.makedirs(output_dir, exist_ok=True)

        # Save JSON next to XML output
        json_file = os.path.join(output_dir, f'{args.system}.json')
        # save as JSON file
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(complete_system, f, ensure_ascii=False, indent=4)
        print(f"Generated: {json_file}")
        # converter.convert_complete_aadl_to_xml(complete_system)
        # converter.save_to_file(output_file)
        # print(f"Generated: {output_file}")
        
        # print error and warning summary
        if aadl_parser.has_errors() or aadl_parser.has_warnings():
            aadl_parser.print_summary()
            
    except Exception as e:
        print(f"An error occurred during execution: {str(e)}")
        if hasattr(aadl_parser, 'print_summary'):
            aadl_parser.print_summary()

if __name__ == '__main__':
    main()