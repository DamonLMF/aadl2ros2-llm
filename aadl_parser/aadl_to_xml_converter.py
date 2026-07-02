"""aadl_to_xml_converter.py

Takes the hierarchical Python representation produced by ``aadl_parser`` and
serialises it into a well-structured XML document.  Each AADL construct is
mapped to an XML element with attributes that mirror its key fields.
"""

import os
from typing import Dict
import xml.etree.ElementTree as ET
from xml.dom import minidom

class AADLToXMLConverter:
    """
    Utility class that transforms parsed AADL dictionaries into XML.
    """
    def __init__(self):
        self.root = None

    def convert_port(self, port_info: Dict) -> ET.Element:
        """
        Convert an AADL port dictionary to an XML element.

        :param port_info: Dictionary containing port information.
        :return: XML element representing the port.
        """
        port = ET.Element('port')
        port.set('name', str(port_info.get('name', '').lower()))
        port.set('direction', str(port_info.get('direction', '').lower()))
        port.set('type', str(port_info.get('port_kind', '').lower()))
        if 'initial_value' in port_info and port_info['initial_value']:
            port.set('initial_value', str(port_info.get('initial_value', '')))

        if 'data_type' in port_info and port_info['data_type']:
            data_type = port_info['data_type']
            data_type_elem = ET.SubElement(port, 'data_type')
            
            if isinstance(data_type, dict):
                # Set basic data type properties
                data_type_elem.set('name', str(data_type.get('name', '')))
                data_type_elem.set('package', str(data_type.get('package', '')))
                
                # Add properties if available
                if 'properties' in data_type and data_type['properties']:
                    props_elem = ET.SubElement(data_type_elem, 'properties')
                    for prop in data_type['properties']:
                        if isinstance(prop, dict):
                            prop_elem = self.convert_property(prop)
                            props_elem.append(prop_elem)
                
                # Add subcomponents if available
                if 'subcomponents' in data_type and data_type['subcomponents']:
                    subcomponents = data_type['subcomponents']
                    
                    if isinstance(subcomponents, list) and subcomponents:
                        subcomps_elem = ET.SubElement(data_type_elem, 'subcomponents')
                        
                        for subcomp in subcomponents:
                            if isinstance(subcomp, dict):
                                subcomp_elem = ET.SubElement(subcomps_elem, 'subcomponent')
                                subcomp_elem.set('name', str(subcomp.get('name', '')))
                                subcomp_elem.set('type', str(subcomp.get('type', '').lower()))
                if 'extends' in data_type and data_type['extends']:
                    data_type_elem.set('extends', data_type['extends'])
        return port
        
    def convert_property(self, prop_info: Dict) -> ET.Element:
        """
        Convert an AADL property dictionary to an XML element.

        :param prop_info: Dictionary containing property information.
        :return: XML element representing the property.
        """
        prop = ET.Element('property')
        prop.set('name', str(prop_info.get('name', '')))
        if 'package' in prop_info:
            prop.set('package', prop_info['package'])
        prop.set('value', str(prop_info.get('value', '')))
        if 'source_code' in prop_info:
            prop.set('source_code', prop_info['source_code'])
        return prop
        
    def convert_flow(self, flow_info: Dict) -> ET.Element:
        """
        Convert an AADL flow dictionary to an XML element.

        :param flow_info: Dictionary containing flow information.
        :return: XML element representing the flow.
        """
        flow = ET.Element('flow')
        flow.set('name', str(flow_info.get('name', '')))
        flow.set('type', str(flow_info.get('type', '').lower()))
        flow.set('source', str(flow_info.get('source', '')))
        flow.set('destination', str(flow_info.get('destination', '')))
        flow.set('path', str(flow_info.get('path', '')))
        if flow_info['properties'] is not None:
            for prop in flow_info['properties']:
                prop_elem = ET.Element('property')
                prop_elem.set('name', str(prop.get('property_name', '')))
                prop_elem.set('value', str(prop.get('property_value', '')))
                flow.append(prop_elem)
        return flow
        
    def convert_annex(self, annex_info: Dict) -> ET.Element:
        """
        Convert an AADL annex dictionary to an XML element.

        :param annex_info: Dictionary containing annex information.
        :return: XML element representing the annex.
        """
        annex = ET.Element('annex')
        annex.set('name', str(annex_info.get('name', '')))
        annex.set('body', str(annex_info.get('body', '')))
        return annex
 
    def convert_connection(self, conn_info: Dict) -> ET.Element:
        """
        Convert an AADL connection dictionary to an XML element.

        :param conn_info: Dictionary containing connection information.
        :return: XML element representing the connection.
        """
        conn = ET.Element('connection')
        conn.set('name', str(conn_info.get('name', '').lower()))
        conn.set('type', str(conn_info.get('type', '').lower()))
        conn.set('source', str(conn_info.get('source', '').lower()))
        conn.set('destination', str(conn_info.get('destination', '').lower()))
        if conn_info.get('modes') and conn_info['modes']:
            conn.set('modes', str(conn_info.get('modes', '').lower()))
        return conn

    def convert_modes(self, mode_info: Dict) -> ET.Element:
        """
        Convert an AADL mode dictionary to an XML element.

        :param mode_info: Dictionary containing mode information.
        :return: XML element representing the mode.
        """
        if 'transition' not in mode_info:
            mode = ET.Element('mode')
            mode.set('name', str(mode_info.get('name', '')))
            mode.set('type', str(mode_info.get('type', '').lower()))
            return mode
        else:
            transition = ET.Element('transition')
            transition.set('transition', str(mode_info.get('transition', '')))
            return transition

    def convert_call(self, call_info: Dict) -> ET.Element:
        """
        Convert an AADL call dictionary to an XML element.

        :param call_info: Dictionary containing call information.
        :return: XML element representing the call.
        """
        call = ET.Element('call')
        call.set('call_name', str(call_info.get('call_name', '')))
        call.set('subprogram_name', str(call_info.get('subprogram_name', '')))
        call.set('subprogram_impl', str(call_info.get('subprogram_impl', '')))
        call.set('subprogram_package', str(call_info.get('subprogram_package', '')))
        subprogram = ET.SubElement(call, 'subprogram')
        if 'subprogram_port' in call_info and call_info['subprogram_port'] is not None:
            ports_elem = ET.SubElement(subprogram, 'ports')
            for port in call_info['subprogram_port']:
                ports_elem.append(self.convert_port(port))
        if 'subprogram_properties' in call_info and call_info['subprogram_properties'] is not None:
            props_elem = ET.SubElement(subprogram, 'properties')
            for subprogram_property in call_info['subprogram_properties']:
                props_elem.append(self.convert_property(subprogram_property))
        return call        

    def convert_component(self, component_info: Dict) -> ET.Element:
        """
        Convert an AADL component dictionary to an XML element.

        :param component_info: Dictionary containing component information.
        :return: XML element representing the component.
        """
        component = ET.Element('component')
        component.set('name', str(component_info.get('name', '').lower()))
        if component_info.get('implementation'):
            component.set('implementation', str(component_info.get('implementation', '').lower()))
        component.set('category', str(component_info.get('category', '').lower()))
        component.set('package', str(component_info.get('package', '')))
        
        # set subcomp_modes attribute
        if component_info.get('subcomp_modes'):
            component.set('subcomp_modes', str(component_info.get('subcomp_modes', '')))

        # add extends
        if component_info.get('extends'):
            extends = ET.SubElement(component, 'extends') 
            for extend in component_info.get('extends'):
                extends.set('extends_name', str(extend.get('name', '')))
                extends.set('package', str(extend.get('package', '')))

        # add subcomponents
        if component_info.get('subcomponents'):
            subcomponents_elem = ET.SubElement(component, 'subcomponents')
            for subcomponent in component_info['subcomponents']:
                subcomponents_elem.append(self.convert_component(subcomponent))
        
        # add ports
        if component_info.get('ports') or component_info.get('extends'):
            # Collect all ports (own + extends) and remove duplicates
            all_ports = []
            if component_info.get('ports'):
                all_ports.extend(component_info['ports'])
            if component_info.get('extends'):
                for extend in component_info['extends']:
                    if extend.get('ports'):
                        all_ports.extend(extend.get('ports'))
            # Remove duplicate ports
            seen = set()
            unique_ports = []
            for port in all_ports:
                # Create a unique key based on port name, direction, and type
                port_key = (port.get('name', '').lower(), port.get('direction', '').lower(), port.get('port_kind', '').lower())
                if port_key not in seen:
                    seen.add(port_key)
                    unique_ports.append(port)
            # Add unique ports to XML
            if unique_ports:
                ports_elem = ET.SubElement(component, 'ports')
                for port in unique_ports:
                    port_elem = self.convert_port(port)
                    ports_elem.append(port_elem)
        
        # add properties
        if component_info.get('properties') or component_info.get('extends'):
            props_elem = ET.SubElement(component, 'properties')
            for prop_info in component_info['properties']:
                props_elem.append(self.convert_property(prop_info))
            for extend in component_info.get('extends'):
                for prop_info in extend.get('properties'):
                    props_elem.append(self.convert_property(prop_info))
        
        # add flows
        if component_info.get('flows'):
            flows_elem = ET.SubElement(component, 'flows')
            for flow_info in component_info['flows']:
                flows_elem.append(self.convert_flow(flow_info))
        
        # add annexes
        if component_info.get('annexes'):
            annexes_elem = ET.SubElement(component, 'annexes')
            for annex_info in component_info['annexes']:
                annexes_elem.append(self.convert_annex(annex_info))
        
        # add connections
        if component_info.get('connections'):
            connections_elem = ET.SubElement(component, 'connections')
            for conn_info in component_info['connections']:
                connections_elem.append(self.convert_connection(conn_info))
        
        # add modes
        if component_info.get('modes'):
            modes_elem = ET.SubElement(component, 'modes')
            for mode_info in component_info['modes']:
                modes_elem.append(self.convert_modes(mode_info))

        # add calls
        if component_info.get('calls'):
            calls_elem = ET.SubElement(component, 'calls')
            for call_info in component_info['calls']:
                calls_elem.append(self.convert_call(call_info))
        
        return component

    def convert_complete_aadl_to_xml(self, systems) -> str:
        """
        convert a list of AADL system data to a single XML string,
        each system implementation is a separate XML node.
        
        Args:
            systems: AADL system data list, each system corresponds to one implementation
            
        Returns:
            XML string containing all system implementations
        """
        # Create a wrapper root node to contain all systems
        systems_root = ET.Element('AADL_models')
        
        # If the input is not a list, convert it to a list
        if not isinstance(systems, list):
            systems = [systems]
            
        for system_data in systems:
            # Create a separate XML node for each system implementation
            system_elem = ET.SubElement(systems_root, system_data['category'].lower())  # Create system node
            system_elem.set('name', system_data['name'])  # Set system name
            system_elem.set('package', system_data['package'])  # Set package name
            system_elem.set('category', system_data['category'].lower())  # Set category
            system_elem.set('implementation', system_data['implementation'])  # Set implementation name
            # Add extends
            if len(system_data.get('extends')) > 0:
                extends = ET.SubElement(system_elem, 'extends') 
                for extend in system_data.get('extends'):
                    if extend is not None:
                        extends.set('extend_name', str(extend.get('name', '')))
                        extends.set('package', str(extend.get('package', '')))
                        extends.append(self.convert_component(extend))

            # Add ports
            if system_data.get('ports'):
                ports_elem = ET.SubElement(system_elem, 'ports')
                for port_info in system_data['ports']:
                    ports_elem.append(self.convert_port(port_info))
            
            # Add properties
            if system_data.get('properties'):
                props_elem = ET.SubElement(system_elem, 'properties')
                for prop_info in system_data['properties']:
                    props_elem.append(self.convert_property(prop_info))
            
            # Add subcomponents
            if system_data.get('subcomponents'):
                subcomps_elem = ET.SubElement(system_elem, 'subcomponents')
                added_components = set() 
                
                for subcomp in system_data['subcomponents']:
                    comp_id = (subcomp.get('name', ''), subcomp.get('implementation', ''))
                    if comp_id in added_components:
                        continue
                    
                    added_components.add(comp_id)
                    subcomps_elem.append(self.convert_component(subcomp))

            # Add connections
            if system_data.get('connections'):
                conns_elem = ET.SubElement(system_elem, 'connections')
                for conn_info in system_data['connections']:
                    conns_elem.append(self.convert_connection(conn_info))
            
             # Add flows
            if system_data.get('flows'):
                flows_elem = ET.SubElement(system_elem, 'flows')
                for flow_info in system_data['flows']:
                    flows_elem.append(self.convert_flow(flow_info))

            # Add annexes
            if system_data.get('annexes'):
                annexes_elem = ET.SubElement(system_elem, 'annexes')
                for annex_info in system_data['annexes']:
                    annexes_elem.append(self.convert_annex(annex_info))

        # Record systems_root as the current root element
        self.root = systems_root
            
        # Format XML output
        xml_str = ET.tostring(systems_root, encoding='unicode')  # convert XML tree to string
        parsed_xml = minidom.parseString(xml_str)  # parse string using minidom
        self.xml_str = parsed_xml.toprettyxml(indent="  ")  # format XML
        return self.xml_str

    def save_to_file(self, filename: str):
        """Save XML to file"""
        if self.root is not None:
            # Ensure the directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)  # create directory (if it doesn't exist)
            xml_str = ET.tostring(self.root, encoding='unicode')  # convert XML tree to string
            parsed_xml = minidom.parseString(xml_str)  # parse string using minidom
            
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(parsed_xml.toprettyxml(indent="  "))  # write formatted XML to file
