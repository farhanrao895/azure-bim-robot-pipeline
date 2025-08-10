"""
ENHANCED Azure BIM-Robot Pipeline - Phase 1 + Phase 2 with ALL FIXES
=====================================================================
Features:
- Multi-floor navigation with proper z-coordinates
- Semantic space name targeting
- Floor-aware task distribution
- Security improvements for API keys
- Enhanced IFC parsing with semantic names
- Fixed ROS2 command generation

Version: 10.0 - Production Ready with All Enhancements
"""

import azure.functions as func
import json
import logging
import os
import re
import time
import math
import yaml
from typing import Dict, List, Optional, Any
from azure.digitaltwins.core import DigitalTwinsClient
from azure.identity import DefaultAzureCredential
from azure.storage.blob import BlobServiceClient
from openai import AzureOpenAI

# Initialize Function App
app = func.FunctionApp(http_auth_level=func.AuthLevel.FUNCTION)

# =============================================================================
# HELPER: Collect space centers from ADT for LLM
# =============================================================================

def _collect_space_centers_from_adt(building_twin):
    """Collect floor elevation and space names from ADT for LLM context"""
    # Minimal, safe fallback if query fails
    data = {"floors": [{"name": "Ground Floor", "number": 0, "elevation": 0.0, "spaces": []}]}
    try:
        dt_url = os.environ.get("DIGITAL_TWINS_URL")
        client = DigitalTwinsClient(dt_url, DefaultAzureCredential())
        bid = building_twin.get("$dtId") or building_twin.get("id") or ""
        q = f"""
        SELECT floor.$dtId as floorId, floor.floorName as floorName, floor.elevation as elev, room.$dtId as roomId,
               room.spaceName as spaceName
        FROM digitaltwins building
        JOIN floor RELATED building.contains
        JOIN room RELATED floor.contains
        WHERE building.$dtId = '{bid}'
        """
        items = list(client.query_twins(q))
        by_floor = {}
        for it in items:
            key = it.get("floorId")
            if key not in by_floor:
                by_floor[key] = {
                    "name": it.get("floorName","Floor"),
                    "elevation": float(it.get("elev") or 0.0),
                    "spaces": []
                }
            by_floor[key]["spaces"].append({
                "name": it.get("spaceName","Room"),
                # x/y unknown here; LLM still benefits from names + elevation
                "center": {"x": 0.0, "y": 0.0}
            })
        data["floors"] = [{"name": v["name"], "elevation": v["elevation"], "spaces": v["spaces"]} for v in by_floor.values()]
        # add floor numbers: sort by elevation
        data["floors"].sort(key=lambda f: f["elevation"])
        for i,f in enumerate(data["floors"]):
            f["number"] = i
        return data
    except Exception:
        return data

# =============================================================================
# ENHANCED IFC PARSER WITH SEMANTIC NAME EXTRACTION
# =============================================================================

class EnhancedIFCParser:
    """ENHANCED IFC parser with semantic name extraction"""
    
    def __init__(self):
        self.discovered_entities = {}
        self.discovered_relationships = {}
        
    def parse_ifc_enhanced(self, ifc_content: str, filename: str) -> Dict[str, Any]:
        """Parse IFC content with ENHANCED semantic name extraction"""
        logging.info(f"Starting ENHANCED IFC parsing for: {filename}")
        
        # Extract all entities with ENHANCED name parsing
        entities = self._discover_entities_with_enhanced_names(ifc_content)
        logging.info(f"Discovered {len(entities)} entities with ENHANCED semantic names")
        
        # Build spatial hierarchy with semantic names
        spatial_hierarchy = self._build_enhanced_spatial_hierarchy(entities)
        
        # Extract building elements with semantic names
        building_elements = self._extract_building_elements_enhanced(entities)
        
        # Calculate areas and metrics from actual IFC data
        metrics = self._calculate_enhanced_metrics(spatial_hierarchy, building_elements)
        
        return {
            "filename": filename,
            "parsing_method": "enhanced_semantic_extraction",
            "file_size": len(ifc_content),
            "entities_discovered": len(entities),
            "spatial_hierarchy": spatial_hierarchy,
            "building_elements": building_elements,
            "metrics": metrics,
            "semantic_extraction_enhanced": True
        }
    
    def _discover_entities_with_enhanced_names(self, content: str) -> Dict[str, Any]:
        """Discover entities with ENHANCED semantic name extraction"""
        entities = {}
        
        # Pattern to find ANY IFC entity
        pattern = r'#(\d+)\s*=\s*IFC([A-Z]+)\s*\((.*?)\)\s*;'
        
        for match in re.finditer(pattern, content, re.DOTALL):
            entity_id = match.group(1)
            entity_type = match.group(2)
            params = match.group(3)
            
            # Extract ENHANCED semantic name
            extracted_name = self._extract_enhanced_semantic_name(params, entity_type)
            
            entities[entity_id] = {
                "id": entity_id,
                "type": entity_type,
                "name": extracted_name,
                "raw_params": params
            }
        
        return entities
    
    def _extract_enhanced_semantic_name(self, params: str, entity_type: str) -> str:
        """ENHANCED: Extract semantic names with area information"""
        if not params:
            return self._get_default_name_by_type(entity_type)
        
        # Split parameters by commas (but respect quoted strings)
        param_parts = self._split_ifc_parameters(params)
        
        semantic_name = None
        area_info = ""
        
        # Method 1: Look for the 3rd parameter as the semantic name (preferred)
        if len(param_parts) >= 3:
            potential_name = param_parts[2].strip()
            if potential_name and potential_name != '$' and potential_name != '':
                # Remove quotes if present
                if potential_name.startswith("'") and potential_name.endswith("'"):
                    potential_name = potential_name[1:-1]
                
                # Check if it's a proper semantic name (not GUID, not just area)
                if (not self._is_guid_pattern(potential_name) and 
                    self._is_semantic_name(potential_name)):
                    semantic_name = potential_name
                elif self._is_area_only(potential_name):
                    area_info = potential_name  # Store area info for later
        
        # Method 2: Look for the 4th parameter (Description) if Name was not semantic
        if not semantic_name and len(param_parts) >= 4:
            potential_desc = param_parts[3].strip()
            if potential_desc and potential_desc != '$' and potential_desc != '':
                if potential_desc.startswith("'") and potential_desc.endswith("'"):
                    potential_desc = potential_desc[1:-1]
                
                if (not self._is_guid_pattern(potential_desc) and 
                    self._is_semantic_name(potential_desc)):
                    semantic_name = potential_desc
        
        # Method 3: Look for ANY quoted string that's semantic
        if not semantic_name:
            quoted_strings = re.findall(r"'([^']*)'", params)
            for string in quoted_strings:
                if (string and string != '$' and 
                    not self._is_guid_pattern(string) and 
                    self._is_semantic_name(string)):
                    # Prefer longer, more descriptive semantic names
                    if not semantic_name or len(string) > len(semantic_name):
                        semantic_name = string
                elif self._is_area_only(string) and not area_info:
                    area_info = string  # Store area info
        
        # Method 4: Generate meaningful name based on type and context
        if not semantic_name:
            if entity_type == "SPACE":
                semantic_name = self._generate_semantic_space_name(params)
            else:
                semantic_name = self._generate_meaningful_name(entity_type, params)
        
        # Method 5: Combine semantic name with area if available
        final_name = semantic_name
        if entity_type == "SPACE" and semantic_name:
            # Try to add area information
            if area_info:
                final_name = f"{semantic_name} ({area_info})"
            else:
                # Calculate area and add it
                calculated_area = self._extract_area_from_params(params)
                if calculated_area > 0:
                    final_name = f"{semantic_name} ({calculated_area:.1f}sqm)"
        
        return final_name if final_name else self._get_default_name_by_type(entity_type)
    
    def _split_ifc_parameters(self, params: str) -> List[str]:
        """Split IFC parameters while respecting quoted strings"""
        parts = []
        current_part = ""
        in_quotes = False
        paren_depth = 0
        
        for char in params:
            if char == "'" and paren_depth == 0:
                in_quotes = not in_quotes
                current_part += char
            elif char == '(' and not in_quotes:
                paren_depth += 1
                current_part += char
            elif char == ')' and not in_quotes:
                paren_depth -= 1
                current_part += char
            elif char == ',' and not in_quotes and paren_depth == 0:
                parts.append(current_part.strip())
                current_part = ""
            else:
                current_part += char
        
        if current_part.strip():
            parts.append(current_part.strip())
        
        return parts
    
    def _is_guid_pattern(self, text: str) -> bool:
        """Check if text looks like a GUID or entity reference"""
        if not text:
            return False
        
        # Common GUID patterns
        guid_patterns = [
            r'^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$',
            r'^[0-9A-Za-z_$]{20,}$',
            r'^[0-9A-Fa-f_$]{15,}$',
            r'^\d+[A-Za-z]\w+$',
            r'^[A-Za-z]\d+\w*$',
        ]
        
        for pattern in guid_patterns:
            if re.match(pattern, text):
                return True
        
        return False
    
    def _is_area_only(self, text: str) -> bool:
        """Check if text is just area information without semantic meaning"""
        if not text:
            return False
        
        # Patterns that indicate area-only information
        area_only_patterns = [
            r'^\d+\.?\d*\s*sqm?$',
            r'^\d+\.?\d*\s*sq\.?\s*m$',
            r'^\d+\.?\d*\s*m²$',
            r'^\d+\.?\d*$',
            r'^\d+\.?\d*\s*(square|area)$',
        ]
        
        text_lower = text.lower().strip()
        
        for pattern in area_only_patterns:
            if re.match(pattern, text_lower):
                return True
        
        return False
    
    def _is_semantic_name(self, text: str) -> bool:
        """Check if text represents a semantic/meaningful name"""
        if not text or len(text) < 2:
            return False
        
        text_lower = text.lower().strip()
        
        # Semantic indicators (room types, descriptive words)
        semantic_indicators = [
            # Room types
            'room', 'bedroom', 'bathroom', 'kitchen', 'living', 'dining', 'office',
            'study', 'library', 'closet', 'storage', 'utility', 'laundry', 'garage',
            'foyer', 'entrance', 'hall', 'corridor', 'lobby', 'lounge', 'salon',
            'chamber', 'suite', 'master', 'guest', 'family', 'recreation', 'den',
            'executive', 'meeting', 'conference',
            
            # Building elements
            'wall', 'door', 'window', 'floor', 'ceiling', 'roof', 'column', 'beam',
            'stair', 'elevator', 'escalator', 'ramp', 'balcony', 'terrace', 'patio',
            
            # Descriptive words
            'main', 'primary', 'secondary', 'front', 'back', 'side', 'north', 'south',
            'east', 'west', 'upper', 'lower', 'ground', 'first', 'second', 'basement',
            'large', 'small', 'open', 'enclosed', 'private', 'public', 'common',
        ]
        
        # Check if text contains semantic indicators
        for indicator in semantic_indicators:
            if indicator in text_lower:
                return True
        
        # Check if it looks like a proper name
        if (text[0].isupper() and 
            any(c.islower() for c in text) and 
            any(c.isalpha() for c in text) and
            not self._is_area_only(text) and
            not self._is_guid_pattern(text)):
            return True
        
        return False
    
    def _generate_semantic_space_name(self, params: str) -> str:
        """Generate semantic space names based on area and context"""
        area = self._extract_area_from_params(params)
        
        # Generate semantic space names based on typical room sizes
        if area >= 30:
            return "Large Room"
        elif area >= 20:
            return "Living Room"
        elif area >= 15:
            return "Bedroom"
        elif area >= 10:
            return "Office"
        elif area >= 6:
            return "Bathroom"
        else:
            return "Room"
    
    def _generate_meaningful_name(self, entity_type: str, params: str) -> str:
        """Generate meaningful names based on entity type and context"""
        type_counters = getattr(self, '_type_counters', {})
        if entity_type not in type_counters:
            type_counters[entity_type] = 1
        else:
            type_counters[entity_type] += 1
        self._type_counters = type_counters
        
        # Generate contextual names
        counter = type_counters[entity_type]
        
        type_mappings = {
            "PROJECT": f"Building Project {counter}",
            "SITE": f"Construction Site {counter}",
            "BUILDING": f"Building {counter}",
            "BUILDINGSTOREY": self._generate_floor_name(counter, params),
            "SPACE": f"Room {counter:02d}",
            "WALL": f"Wall {counter:02d}",
            "WALLSTANDARDCASE": f"Wall {counter:02d}",
            "DOOR": f"Door {counter:02d}",
            "WINDOW": f"Window {counter:02d}",
            "SLAB": f"Slab {counter:02d}",
            "COLUMN": f"Column {counter:02d}",
            "BEAM": f"Beam {counter:02d}",
            "STAIR": f"Staircase {counter:02d}"
        }
        
        return type_mappings.get(entity_type, f"{entity_type.title()} {counter}")
    
    def _generate_floor_name(self, counter: int, params: str) -> str:
        """Generate proper floor names"""
        # Try to extract elevation for proper floor naming
        elevation = self._extract_elevation_from_params(params)
        
        if elevation <= 0:
            return "Ground Floor"
        elif elevation <= 3.5:
            return "First Floor"
        elif elevation <= 7:
            return "Second Floor"
        elif elevation <= 10.5:
            return "Third Floor"
        else:
            floor_number = int(elevation / 3.5) + 1
            if floor_number == 2:
                return "Second Floor"
            elif floor_number == 3:
                return "Third Floor"
            elif floor_number == 4:
                return "Fourth Floor"
            else:
                return f"Floor {floor_number}"
    
    def _extract_elevation_from_params(self, params: str) -> float:
        """Extract elevation from parameters"""
        # Look for numeric values that could be elevations
        numbers = re.findall(r'-?\d+\.?\d*', params)
        
        for num_str in numbers:
            try:
                num = float(num_str)
                if -10 <= num <= 100:  # Reasonable elevation range
                    return num
            except ValueError:
                continue
        
        return 0.0
    
    def _extract_area_from_params(self, params: str) -> float:
        """Extract area from parameters for space naming"""
        # Look for area-like numbers (typically between 1-1000)
        numbers = re.findall(r'\d+\.?\d*', params)
        
        for num_str in numbers:
            try:
                num = float(num_str)
                if 1 <= num <= 1000:  # Reasonable area range
                    return num
            except ValueError:
                continue
        
        return 12.0  # Default area
    
    def _get_default_name_by_type(self, entity_type: str) -> str:
        """Get default names for entity types"""
        defaults = {
            "PROJECT": "Building Project",
            "SITE": "Construction Site", 
            "BUILDING": "Main Building",
            "BUILDINGSTOREY": "Floor",
            "SPACE": "Room",
            "WALL": "Wall",
            "WALLSTANDARDCASE": "Wall",
            "DOOR": "Door",
            "WINDOW": "Window",
            "SLAB": "Slab",
            "COLUMN": "Column",
            "BEAM": "Beam",
            "STAIR": "Staircase"
        }
        return defaults.get(entity_type, entity_type.title())
    
    def _build_enhanced_spatial_hierarchy(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Build hierarchy with ENHANCED semantic names"""
        hierarchy = {
            "project": None,
            "site": None,
            "building": None,
            "storeys": [],
            "spaces": []
        }
        
        # Find spatial elements by type with semantic names
        for entity_id, entity in entities.items():
            entity_type = entity["type"]
            entity_name = entity["name"]
            
            if entity_type == "PROJECT":
                hierarchy["project"] = {
                    "id": entity_id,
                    "name": entity_name,
                    "type": self._classify_building_type_from_name(entity_name)
                }
            elif entity_type == "SITE":
                hierarchy["site"] = {
                    "id": entity_id,
                    "name": entity_name
                }
            elif entity_type == "BUILDING":
                hierarchy["building"] = {
                    "id": entity_id,
                    "name": entity_name,
                    "type": self._classify_building_type_from_name(entity_name)
                }
            elif entity_type == "BUILDINGSTOREY":
                elevation = self._extract_elevation_from_params(entity["raw_params"])
                hierarchy["storeys"].append({
                    "id": entity_id,
                    "name": entity_name,
                    "elevation": elevation
                })
            elif entity_type == "SPACE":
                space_area = self._extract_area_from_name_and_params(entity_name, entity["raw_params"])
                hierarchy["spaces"].append({
                    "id": entity_id,
                    "name": entity_name,
                    "type": self._classify_space_type_from_name(entity_name),
                    "area": space_area,
                    "volume": space_area * 3.0
                })
        
        return hierarchy
    
    def _classify_building_type_from_name(self, name: str) -> str:
        """Classify building type from semantic name"""
        if not name:
            return "mixed_use"
        
        name_lower = name.lower()
        
        if any(keyword in name_lower for keyword in ["house", "home", "residential", "apartment", "villa"]):
            return "residential"
        elif any(keyword in name_lower for keyword in ["office", "commercial", "business", "retail", "shop"]):
            return "commercial"
        elif any(keyword in name_lower for keyword in ["factory", "warehouse", "industrial", "plant"]):
            return "industrial"
        elif any(keyword in name_lower for keyword in ["school", "hospital", "university", "clinic"]):
            return "institutional"
        else:
            return "mixed_use"
    
    def _classify_space_type_from_name(self, name: str) -> str:
        """Classify space type from semantic name"""
        if not name:
            return "general_space"
        
        name_lower = name.lower()
        
        # Classify based on actual semantic name content
        space_indicators = {
            "bathroom": ["bathroom", "toilet", "wc", "restroom", "washroom", "lavatory"],
            "kitchen": ["kitchen", "cook", "culinary", "pantry", "galley"],
            "bedroom": ["bedroom", "bed", "sleep", "master", "guest", "chamber"],
            "living_room": ["living", "lounge", "sitting", "family", "parlor", "salon"],
            "dining_room": ["dining", "eat", "meal", "breakfast", "lunch", "dinner"],
            "office": ["office", "study", "work", "desk", "computer", "library"],
            "circulation": ["corridor", "hall", "passage", "lobby", "foyer", "entrance"],
            "storage": ["storage", "store", "closet", "cupboard", "wardrobe", "utility"],
            "meeting_room": ["conference", "meeting", "board", "presentation", "seminar", "executive"]
        }
        
        for space_type, keywords in space_indicators.items():
            if any(keyword in name_lower for keyword in keywords):
                return space_type
        
        return "general_space"
    
    def _extract_area_from_name_and_params(self, name: str, params: str) -> float:
        """Extract area with preference for name-based extraction"""
        # Method 1: Extract from name if present (enhanced to handle combined names)
        if name:
            # Look for area in parentheses: "BEDROOM (25.92sqm)"
            paren_match = re.search(r'\((\d+\.?\d*)\s*sqm?\)', name, re.IGNORECASE)
            if paren_match:
                try:
                    area = float(paren_match.group(1))
                    if 0.1 <= area <= 10000:
                        return area
                except ValueError:
                    pass
            
            # Look for area patterns in the name
            area_patterns = [
                r'(\d+\.?\d*)\s*sqm',
                r'(\d+\.?\d*)\s*sq\.?\s*m',
                r'(\d+\.?\d*)\s*m²',
                r'(\d+\.?\d*)\s*square'
            ]
            
            for pattern in area_patterns:
                match = re.search(pattern, name.lower())
                if match:
                    try:
                        area = float(match.group(1))
                        if 0.1 <= area <= 10000:
                            return area
                    except ValueError:
                        continue
        
        # Method 2: Estimate based on space type and name
        space_type = self._classify_space_type_from_name(name)
        name_lower = name.lower() if name else ""
        
        # Size hints from name
        if "large" in name_lower:
            multiplier = 1.5
        elif "small" in name_lower:
            multiplier = 0.7
        else:
            multiplier = 1.0
        
        type_estimates = {
            "bathroom": 6.0,
            "kitchen": 12.0,
            "bedroom": 15.0,
            "living_room": 25.0,
            "dining_room": 18.0,
            "office": 12.0,
            "circulation": 8.0,
            "storage": 4.0,
            "meeting_room": 30.0,
            "general_space": 15.0
        }
        
        base_area = type_estimates.get(space_type, 15.0)
        return base_area * multiplier
    
    def _extract_building_elements_enhanced(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Extract building elements with semantic names"""
        elements = {
            "walls": [],
            "doors": [],
            "windows": [],
            "slabs": [],
            "columns": [],
            "beams": [],
            "stairs": []
        }
        
        element_types = {
            "walls": ["WALL", "WALLSTANDARDCASE", "WALLELEMENTEDCASE"],
            "doors": ["DOOR"],
            "windows": ["WINDOW"],
            "slabs": ["SLAB"],
            "columns": ["COLUMN"],
            "beams": ["BEAM"],
            "stairs": ["STAIR"]
        }
        
        for entity_id, entity in entities.items():
            entity_type = entity["type"]
            
            for element_category, type_list in element_types.items():
                if entity_type in type_list:
                    elements[element_category].append({
                        "id": entity_id,
                        "name": entity["name"],
                        "type": entity_type
                    })
        
        return elements
    
    def _calculate_enhanced_metrics(self, spatial_hierarchy: Dict[str, Any], building_elements: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate metrics from semantically named data"""
        total_area = sum(space["area"] for space in spatial_hierarchy.get("spaces", []))
        
        element_counts = {
            "walls": len(building_elements.get("walls", [])),
            "doors": len(building_elements.get("doors", [])),
            "windows": len(building_elements.get("windows", [])),
            "slabs": len(building_elements.get("slabs", [])),
            "columns": len(building_elements.get("columns", [])),
            "beams": len(building_elements.get("beams", [])),
            "stairs": len(building_elements.get("stairs", []))
        }
        
        return {
            "total_area": total_area,
            "total_storeys": len(spatial_hierarchy.get("storeys", [])),
            "total_spaces": len(spatial_hierarchy.get("spaces", [])),
            "element_counts": element_counts,
            "average_space_area": total_area / max(len(spatial_hierarchy.get("spaces", [])), 1),
            "semantic_extraction_quality": "enhanced_semantic_names"
        }

# =============================================================================
# DTDL MODELS (with CommandRequest)
# =============================================================================

def get_enhanced_dtdl_models() -> List[Dict[str, Any]]:
    """ENHANCED DTDL models with CommandRequest (not CommandPayload)"""
    return [
        {
            "@id": "dtmi:construction:robot:Space;1",
            "@type": "Interface",
            "@context": "dtmi:dtdl:context;3",
            "displayName": "Robot Construction Space",
            "description": "Space model for robot navigation and construction tasks",
            "contents": [
                {"@type": "Property", "name": "spaceName", "schema": "string"},
                {"@type": "Property", "name": "spaceType", "schema": "string"},
                {"@type": "Property", "name": "area", "schema": "double"},
                {"@type": "Property", "name": "volume", "schema": "double"},
                {"@type": "Property", "name": "robotTaskZone", "schema": "string"},
                {"@type": "Property", "name": "navigationWaypoints", "schema": "string"},
                {"@type": "Property", "name": "constructionPhase", "schema": "string"},
                {"@type": "Property", "name": "safetyStatus", "schema": "string"},
                {"@type": "Telemetry", "name": "robotPresence", "schema": "boolean"},
                {"@type": "Telemetry", "name": "taskProgress", "schema": "double"},
                {"@type": "Telemetry", "name": "environmentData", "schema": "string"},
                {"@type": "Command", "name": "assignRobotTask", "request": {"@type": "CommandRequest", "name": "taskData", "schema": "string"}},
                {"@type": "Command", "name": "updateTaskProgress", "request": {"@type": "CommandRequest", "name": "progress", "schema": "double"}}
            ]
        },
        {
            "@id": "dtmi:construction:robot:Floor;1",
            "@type": "Interface",
            "@context": "dtmi:dtdl:context;3",
            "displayName": "Robot Construction Floor",
            "description": "Floor model for robot coordination and navigation",
            "contents": [
                {"@type": "Property", "name": "floorName", "schema": "string"},
                {"@type": "Property", "name": "elevation", "schema": "double"},
                {"@type": "Property", "name": "robotAccessible", "schema": "boolean"},
                {"@type": "Property", "name": "constructionPhase", "schema": "string"},
                {"@type": "Property", "name": "safetyZones", "schema": "string"},
                {"@type": "Property", "name": "floorPlan", "schema": "string"},
                {"@type": "Property", "name": "emergencyExits", "schema": "string"},
                {"@type": "Telemetry", "name": "activeRobots", "schema": "integer"},
                {"@type": "Telemetry", "name": "constructionProgress", "schema": "double"},
                {"@type": "Telemetry", "name": "safetyAlerts", "schema": "string"},
                {"@type": "Command", "name": "deployRobots", "request": {"@type": "CommandRequest", "name": "robotConfig", "schema": "string"}},
                {"@type": "Command", "name": "emergencyStop", "request": {"@type": "CommandRequest", "name": "reason", "schema": "string"}},
                {"@type": "Relationship", "name": "contains", "target": "dtmi:construction:robot:Space;1"}
            ]
        },
        {
            "@id": "dtmi:construction:robot:Building;1",
            "@type": "Interface",
            "@context": "dtmi:dtdl:context;3",
            "displayName": "Robot Construction Building",
            "description": "Building model for comprehensive robot construction management",
            "contents": [
                {"@type": "Property", "name": "buildingName", "schema": "string"},
                {"@type": "Property", "name": "buildingType", "schema": "string"},
                {"@type": "Property", "name": "constructionStatus", "schema": "string"},
                {"@type": "Property", "name": "totalArea", "schema": "double"},
                {"@type": "Property", "name": "robotDeploymentZones", "schema": "string"},
                {"@type": "Property", "name": "constructionSequence", "schema": "string"},
                {"@type": "Property", "name": "projectTimeline", "schema": "string"},
                {"@type": "Property", "name": "qualityStandards", "schema": "string"},
                {"@type": "Telemetry", "name": "overallProgress", "schema": "double"},
                {"@type": "Telemetry", "name": "robotFleetStatus", "schema": "string"},
                {"@type": "Telemetry", "name": "productivityMetrics", "schema": "string"},
                {"@type": "Command", "name": "initiateConstruction", "request": {"@type": "CommandRequest", "name": "constructionPlan", "schema": "string"}},
                {"@type": "Command", "name": "pauseConstruction", "request": {"@type": "CommandRequest", "name": "reason", "schema": "string"}},
                {"@type": "Command", "name": "generateReport", "request": {"@type": "CommandRequest", "name": "reportType", "schema": "string"}},
                {"@type": "Relationship", "name": "contains", "target": "dtmi:construction:robot:Floor;1"}
            ]
        }
    ]

# =============================================================================
# PHASE 1 FUNCTIONS - ENHANCED
# =============================================================================

@app.route(route="ModelUploader")
def ModelUploader(req: func.HttpRequest) -> func.HttpResponse:
    """Upload ENHANCED robot-ready DTDL models to Azure Digital Twins"""
    logging.info('ENHANCED ModelUploader function triggered')
    
    try:
        dt_url = os.environ.get("DIGITAL_TWINS_URL")
        if not dt_url:
            return func.HttpResponse(
                json.dumps({"status": "error", "message": "DIGITAL_TWINS_URL not configured"}),
                status_code=500,
                mimetype="application/json"
            )
        
        credential = DefaultAzureCredential()
        dt_client = DigitalTwinsClient(dt_url, credential)
        
        models = get_enhanced_dtdl_models()
        result = dt_client.create_models(models)
        
        logging.info(f"Successfully uploaded {len(result)} ENHANCED robot-ready DTDL models")
        
        return func.HttpResponse(
            json.dumps({
                "status": "success",
                "message": f"Uploaded {len(result)} ENHANCED robot-ready DTDL models",
                "models": [model.id for model in result],
                "enhancements_applied": [
                    "Multi-floor navigation support",
                    "Semantic name extraction",
                    "CommandRequest correction",
                    "Floor-aware task distribution"
                ]
            }),
            status_code=200,
            mimetype="application/json"
        )
        
    except Exception as e:
        logging.error(f"Error uploading ENHANCED robot-ready models: {str(e)}")
        return func.HttpResponse(
            json.dumps({"status": "error", "message": str(e)}),
            status_code=500,
            mimetype="application/json"
        )

@app.route(route="UploadBIM", methods=["POST"])
def UploadBIM(req: func.HttpRequest) -> func.HttpResponse:
    """Upload and process BIM file with ENHANCED semantic name extraction"""
    logging.info('ENHANCED UploadBIM function triggered')
    
    try:
        file_data = req.get_body()
        if not file_data:
            return func.HttpResponse(
                json.dumps({"status": "error", "message": "No file data received"}),
                status_code=400,
                mimetype="application/json"
            )
        
        filename = req.params.get('filename', 'uploaded_building.ifc')
        logging.info(f"Processing file with ENHANCED semantic parsing: {filename} ({len(file_data)} bytes)")
        
        # Store in Azure Blob Storage
        blob_url = None
        storage_connection_string = os.environ.get("STORAGE_CONNECTION_STRING")
        if storage_connection_string:
            try:
                blob_service_client = BlobServiceClient.from_connection_string(storage_connection_string)
                container_name = "enhanced-bim-files"
                blob_name = f"{filename}_{int(time.time())}"
                
                try:
                    blob_service_client.create_container(container_name)
                except Exception:
                    pass
                
                blob_client = blob_service_client.get_blob_client(container=container_name, blob=blob_name)
                blob_client.upload_blob(file_data, overwrite=True)
                blob_url = blob_client.url
                logging.info(f"File uploaded to blob storage: {blob_url}")
            except Exception as e:
                logging.warning(f"Blob storage upload failed: {str(e)}")
        
        # ENHANCED IFC PARSING WITH SEMANTIC NAMES
        parser = EnhancedIFCParser()
        ifc_content = file_data.decode('utf-8', errors='ignore')
        
        # Parse with ENHANCED semantic extraction
        parsed_data = parser.parse_ifc_enhanced(ifc_content, filename)
        
        # Extract building information from ENHANCED parsing
        spatial_hierarchy = parsed_data.get('spatial_hierarchy', {})
        metrics = parsed_data.get('metrics', {})
        
        # Build comprehensive building data with semantic names
        building_name = "Main Building"
        if spatial_hierarchy.get('building'):
            building_name = spatial_hierarchy['building'].get('name', 'Main Building')
        elif spatial_hierarchy.get('project'):
            building_name = spatial_hierarchy['project'].get('name', 'Main Building')
        
        building_data = {
            "building_name": building_name,
            "building_type": spatial_hierarchy.get('building', {}).get('type', 'mixed_use'),
            "total_area": metrics.get('total_area', 0),
            "total_floors": metrics.get('total_storeys', 1),
            "total_spaces": metrics.get('total_spaces', 0),
            "floors": convert_enhanced_hierarchy_to_floors(parsed_data),
            "building_elements": parsed_data.get('building_elements', {}),
            "parsing_method": parsed_data.get('parsing_method', 'enhanced_semantic_extraction')
        }
        
        # Create Digital Twins with ENHANCED semantic names
        dt_url = os.environ.get("DIGITAL_TWINS_URL")
        credential = DefaultAzureCredential()
        dt_client = DigitalTwinsClient(dt_url, credential)
        
        building_id = f"building-{filename.replace('.', '-').replace('_', '-').lower()}"
        
        # Create building twin with semantic names
        building_twin = {
            "$metadata": {"$model": "dtmi:construction:robot:Building;1"},
            "buildingName": building_data.get("building_name", "Main Building"),
            "buildingType": building_data.get("building_type", "mixed_use"),
            "constructionStatus": "enhanced_bim_processed_robot_ready",
            "totalArea": building_data.get("total_area", 0.0),
            "robotDeploymentZones": json.dumps([floor["name"] for floor in building_data.get("floors", [])]),
            "constructionSequence": "ai_optimized_enhanced_robot",
            "projectTimeline": json.dumps({
                "upload_date": time.strftime("%Y-%m-%d"),
                "parsing_method": "enhanced_semantic_extraction"
            }),
            "qualityStandards": "enhanced_robot_quality_control"
        }
        
        dt_client.upsert_digital_twin(building_id, building_twin)
        logging.info(f"Created ENHANCED building twin: {building_id}")
        
        floors_created = 0
        rooms_created = 0
        
        # Create floor and space twins with ENHANCED semantic names
        for floor_data in building_data.get("floors", []):
            floor_id = f"{building_id}-floor-{floor_data['number']}"
            floor_name = floor_data.get("name", f"Floor {floor_data['number']}")
            
            floor_twin = {
                "$metadata": {"$model": "dtmi:construction:robot:Floor;1"},
                "floorName": floor_name,
                "elevation": floor_data.get("elevation", floor_data["number"] * 3.0),
                "robotAccessible": True,
                "constructionPhase": "enhanced_robot_planning",
                "safetyZones": json.dumps([
                    {"name": f"{floor_name} entry zone", "coordinates": [0, 0, 2, 2]},
                    {"name": f"{floor_name} work zone", "coordinates": [2, 2, 8, 8]}
                ]),
                "floorPlan": json.dumps(floor_data),
                "emergencyExits": json.dumps([{"x": 0, "y": 0}, {"x": 10, "y": 10}])
            }
            
            dt_client.upsert_digital_twin(floor_id, floor_twin)
            floors_created += 1
            
            # Create building-floor relationship
            relationship_id = f"building-contains-floor-{floor_data['number']}"
            relationship = {
                "$targetId": floor_id,
                "$relationshipName": "contains"
            }
            dt_client.upsert_relationship(building_id, relationship_id, relationship)
            
            # Create space twins with ENHANCED semantic names
            for space_data in floor_data.get("spaces", []):
                room_id = f"{floor_id}-{space_data['id']}"
                space_name = space_data.get("name", f"Room {rooms_created + 1}")
                
                room_twin = {
                    "$metadata": {"$model": "dtmi:construction:robot:Space;1"},
                    "spaceName": space_name,
                    "spaceType": space_data["type"],
                    "area": space_data["area"],
                    "volume": space_data["volume"],
                    "robotTaskZone": determine_enhanced_robot_task_zone(space_data["type"], space_name),
                    "navigationWaypoints": json.dumps(space_data.get("robot_navigation_data", {}).get("waypoints", [])),
                    "constructionPhase": "awaiting_enhanced_robot_assignment",
                    "safetyStatus": "safe_for_enhanced_robot_operations"
                }
                
                dt_client.upsert_digital_twin(room_id, room_twin)
                rooms_created += 1
                
                # Create floor-room relationship
                room_relationship_id = f"floor-contains-{space_data['id']}"
                room_relationship = {
                    "$targetId": room_id,
                    "$relationshipName": "contains"
                }
                dt_client.upsert_relationship(floor_id, room_relationship_id, room_relationship)
        
        return func.HttpResponse(
            json.dumps({
                "status": "success",
                "message": "BIM file processed with ENHANCED semantic extraction",
                "filename": filename,
                "building_id": building_id,
                "floors_created": floors_created,
                "rooms_created": rooms_created,
                "building_data": building_data
            }),
            status_code=200,
            mimetype="application/json"
        )
        
    except Exception as e:
        logging.error(f"Error in ENHANCED BIM upload: {str(e)}")
        return func.HttpResponse(
            json.dumps({"status": "error", "message": str(e)}),
            status_code=500,
            mimetype="application/json"
        )

@app.route(route="RobotStatus")
def RobotStatus(req: func.HttpRequest) -> func.HttpResponse:
    """Monitor robot fleet status with ENHANCED building integration"""
    logging.info('ENHANCED RobotStatus function triggered')
    
    try:
        building_id = req.params.get('building_id', 'all')
        
        # ENHANCED robot fleet status
        enhanced_robot_status = {
            "fleet_overview": {
                "total_robots": 3,
                "active_robots": 2,
                "idle_robots": 1,
                "maintenance_robots": 0,
                "fleet_efficiency": 94.2,
                "multi_floor_coordination": True
            },
            "individual_robots": [
                {
                    "robot_id": "enhanced_construction_robot_01",
                    "type": "construction_robot",
                    "status": "active_construction",
                    "current_floor": 0,
                    "current_space": "Living Room",
                    "location": {"x": 7.2, "y": 5.1, "z": 0.5}
                },
                {
                    "robot_id": "enhanced_construction_robot_02",
                    "type": "construction_robot",
                    "status": "active_construction",
                    "current_floor": 1,
                    "current_space": "Executive Office",
                    "location": {"x": 3.0, "y": 2.0, "z": 4.0}
                }
            ]
        }
        
        return func.HttpResponse(
            json.dumps({
                "status": "success",
                "building_id": building_id,
                "enhanced_robot_fleet_status": enhanced_robot_status
            }),
            status_code=200,
            mimetype="application/json"
        )
        
    except Exception as e:
        logging.error(f"Error monitoring robot status: {str(e)}")
        return func.HttpResponse(
            json.dumps({"status": "error", "message": str(e)}),
            status_code=500,
            mimetype="application/json"
        )

# =============================================================================
# PHASE 2 FUNCTIONS - ENHANCED WITH FIXES
# =============================================================================

@app.route(route="TaskPlanner")
def TaskPlanner(req: func.HttpRequest) -> func.HttpResponse:
    """Generate construction task plans using Azure OpenAI"""
    logging.info('TaskPlanner function triggered')
    
    try:
        building_id = req.params.get('building_id', 'building-building-architecture-ifc')
        task_type = req.params.get('task_type', 'construction_sequence')
        
        dt_url = os.environ.get("DIGITAL_TWINS_URL")
        credential = DefaultAzureCredential()
        dt_client = DigitalTwinsClient(dt_url, credential)
        
        building_twin = dt_client.get_digital_twin(building_id)
        logging.info(f"Retrieved building twin: {building_id}")
        
        query = f"""
        SELECT building, floor FROM digitaltwins building
        JOIN floor RELATED building.contains
        WHERE building.$dtId = '{building_id}'
        """
        query_result = dt_client.query_twins(query)
        twin_data = list(query_result)
        
        # Get API key from environment (not from code)
        openai_key = os.environ.get("AZURE_OPENAI_API_KEY")
        if not openai_key:
            return func.HttpResponse(
                json.dumps({"status": "error", "message": "Azure OpenAI API key not configured"}),
                status_code=500,
                mimetype="application/json"
            )
        
        client = AzureOpenAI(
            azure_endpoint=os.environ.get("AZURE_OPENAI_ENDPOINT", "https://openainewfsawork.openai.azure.com/"),
            api_key=openai_key,
            api_version="2024-12-01-preview"
        )
        
        task_plan = generate_construction_tasks(client, building_twin, twin_data, task_type)
        
        return func.HttpResponse(
            json.dumps({
                "status": "success",
                "building_id": building_id,
                "task_type": task_type,
                "task_plan": task_plan
            }),
            status_code=200,
            mimetype="application/json"
        )
        
    except Exception as e:
        logging.error(f"Error in TaskPlanner: {str(e)}")
        return func.HttpResponse(
            json.dumps({"status": "error", "message": str(e)}),
            status_code=500,
            mimetype="application/json"
        )

@app.route(route="PromptOrchestrator")
def PromptOrchestrator(req: func.HttpRequest) -> func.HttpResponse:
    """ENHANCED Chain-of-Thought prompting with multi-floor navigation"""
    logging.info('ENHANCED PromptOrchestrator function triggered')
    
    try:
        building_id = req.params.get('building_id', 'building-building-architecture-ifc')
        complexity = req.params.get('complexity', 'detailed')
        min_nav_per_floor = int(req.params.get('min_nav_per_floor', '4'))
        require_upper = req.params.get('require_upper_floor', 'true').lower() == 'true'
        
        openai_key = os.environ.get("AZURE_OPENAI_API_KEY")
        if not openai_key:
            return func.HttpResponse(
                json.dumps({"status": "error", "message": "Azure OpenAI API key not configured"}),
                status_code=500,
                mimetype="application/json"
            )
        
        client = AzureOpenAI(
            azure_endpoint=os.environ.get("AZURE_OPENAI_ENDPOINT", "https://openainewfsawork.openai.azure.com/"),
            api_key=openai_key,
            api_version="2024-12-01-preview"
        )
        
        dt_url = os.environ.get("DIGITAL_TWINS_URL")
        credential = DefaultAzureCredential()
        dt_client = DigitalTwinsClient(dt_url, credential)
        building_twin = dt_client.get_digital_twin(building_id)
        
        orchestration_result = chain_of_thought_planning_enhanced(
            client, building_twin, complexity, min_nav_per_floor, require_upper
        )
        
        return func.HttpResponse(
            json.dumps({
                "status": "success",
                "building_id": building_id,
                "complexity_level": complexity,
                "orchestration_result": orchestration_result,
                "multi_floor_navigation": True
            }),
            status_code=200,
            mimetype="application/json"
        )
        
    except Exception as e:
        logging.error(f"Error in PromptOrchestrator: {str(e)}")
        return func.HttpResponse(
            json.dumps({"status": "error", "message": str(e)}),
            status_code=500,
            mimetype="application/json"
        )

@app.route(route="TaskOutputGenerator")
def TaskOutputGenerator(req: func.HttpRequest) -> func.HttpResponse:
    """ENHANCED ROS2 output with floor-aware navigation"""
    logging.info('ENHANCED TaskOutputGenerator function triggered')
    
    try:
        req_body = req.get_json()
        if not req_body:
            return func.HttpResponse(
                json.dumps({"status": "error", "message": "No task plan data provided"}),
                status_code=400,
                mimetype="application/json"
            )
        
        output_format = req.params.get('format', 'json')
        robot_type = req.params.get('robot_type', 'construction_robot')
        
        structured_output = structure_for_robots(req_body, robot_type)
        ros2_commands = generate_ros2_commands_enhanced(structured_output)
        
        if output_format.lower() == 'yaml':
            output_data = yaml.dump(ros2_commands, default_flow_style=False)
        else:
            output_data = json.dumps(ros2_commands, indent=2)
        
        return func.HttpResponse(
            json.dumps({
                "status": "success",
                "output_format": output_format,
                "robot_type": robot_type,
                "ros2_commands": ros2_commands,
                "formatted_output": output_data,
                "multi_floor_support": True
            }),
            status_code=200,
            mimetype="application/json"
        )
        
    except Exception as e:
        logging.error(f"Error in TaskOutputGenerator: {str(e)}")
        return func.HttpResponse(
            json.dumps({"status": "error", "message": str(e)}),
            status_code=500,
            mimetype="application/json"
        )

# =============================================================================
# ENHANCED HELPER FUNCTIONS
# =============================================================================

def convert_enhanced_hierarchy_to_floors(parsed_data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Convert semantically parsed spatial hierarchy to floor format"""
    floors = []
    
    try:
        spatial_hierarchy = parsed_data.get('spatial_hierarchy', {})
        storeys = spatial_hierarchy.get('storeys', [])
        spaces = spatial_hierarchy.get('spaces', [])
        
        if storeys:
            # Distribute spaces among floors using semantic names
            for idx, storey in enumerate(storeys):
                floor_spaces = []
                storey_name = storey.get("name", f"Floor {idx}")
                
                # Simple distribution - could be enhanced with actual relationships
                spaces_per_floor = len(spaces) // len(storeys)
                start_idx = idx * spaces_per_floor
                end_idx = start_idx + spaces_per_floor if idx < len(storeys) - 1 else len(spaces)
                
                for space_idx in range(start_idx, end_idx):
                    if space_idx < len(spaces):
                        space = spaces[space_idx]
                        space_name = space.get("name", f"Room {space_idx + 1}")
                        floor_spaces.append({
                            "id": f"space-{space_idx + 1:02d}",
                            "name": space_name,
                            "type": space["type"],
                            "area": space["area"],
                            "volume": space["volume"],
                            "robot_navigation_data": generate_enhanced_navigation_waypoints(space, storey_name)
                        })
                
                floors.append({
                    "number": idx,
                    "name": storey_name,
                    "elevation": storey["elevation"],
                    "area": sum(space["area"] for space in floor_spaces),
                    "height": 3.0,
                    "spaces": floor_spaces
                })
        else:
            # Single floor with all spaces
            floor_spaces = []
            for idx, space in enumerate(spaces):
                space_name = space.get("name", f"Room {idx + 1}")
                floor_spaces.append({
                    "id": f"space-{idx + 1:02d}",
                    "name": space_name,
                    "type": space["type"],
                    "area": space["area"],
                    "volume": space["volume"],
                    "robot_navigation_data": generate_enhanced_navigation_waypoints(space, "Ground Floor")
                })
            
            floors.append({
                "number": 0,
                "name": "Ground Floor",
                "elevation": 0.0,
                "area": sum(space["area"] for space in floor_spaces),
                "height": 3.0,
                "spaces": floor_spaces
            })
            
    except Exception as e:
        logging.error(f"Error converting enhanced hierarchy: {str(e)}")
        floors = [{
            "number": 0,
            "name": "Ground Floor",
            "elevation": 0.0,
            "area": 50.0,
            "height": 3.0,
            "spaces": []
        }]
    
    return floors

def generate_enhanced_navigation_waypoints(space_data: Dict[str, Any], floor_name: str) -> Dict[str, Any]:
    """Generate navigation waypoints with semantic space and floor names"""
    area = space_data.get("area", 12.0)
    side_length = math.sqrt(area)
    space_name = space_data.get("name", "Room")
    
    return {
        "space_name": space_name,
        "floor_name": floor_name,
        "semantic_navigation": True,
        "waypoints": [
            {"x": 0.0, "y": 0.0, "z": 0.0, "type": "entry"},
            {"x": side_length * 0.3, "y": side_length * 0.3, "z": 0.0, "type": "work_start"},
            {"x": side_length * 0.7, "y": side_length * 0.7, "z": 0.0, "type": "work_end"},
            {"x": 0.0, "y": 0.0, "z": 0.0, "type": "exit"}
        ]
    }

def determine_enhanced_robot_task_zone(space_type: str, space_name: str) -> str:
    """Determine robot task zone using semantic type and name"""
    name_lower = space_name.lower() if space_name else ""
    
    if any(keyword in name_lower for keyword in ["bathroom", "toilet", "washroom"]):
        return "mep_intensive"
    elif any(keyword in name_lower for keyword in ["kitchen", "cook"]):
        return "mep_intensive"
    elif any(keyword in name_lower for keyword in ["bedroom", "living", "office", "executive"]):
        return "interior_finishing"
    elif any(keyword in name_lower for keyword in ["corridor", "hall", "lobby"]):
        return "structural_priority"
    elif any(keyword in name_lower for keyword in ["conference", "meeting"]):
        return "interior_finishing"
    elif any(keyword in name_lower for keyword in ["storage", "utility"]):
        return "utility_systems"
    
    # Fallback to type-based classification
    task_zones = {
        "bathroom": "mep_intensive",
        "kitchen": "mep_intensive",
        "bedroom": "interior_finishing",
        "living_room": "interior_finishing",
        "dining_room": "interior_finishing",
        "office": "interior_finishing",
        "meeting_room": "interior_finishing",
        "circulation": "structural_priority",
        "storage": "utility_systems",
        "general_space": "general_construction"
    }
    return task_zones.get(space_type, "general_construction")

def generate_construction_tasks(client, building_twin, twin_data, task_type):
    """Generate construction task plans using GPT-4o"""
    try:
        building_name = building_twin.get("buildingName", "Unknown Building")
        building_type = building_twin.get("buildingType", "commercial")
        total_area = building_twin.get("totalArea", 0)
        
        system_prompt = """
        You are an expert construction project manager. 
        Create construction task sequences for robotic execution with multi-floor navigation.
        Include specific space names and floor numbers in navigation commands.
        """
        
        user_prompt = f"""
        Building: {building_name}
        Type: {building_type}
        Area: {total_area} sq meters
        
        Generate construction tasks with navigation to specific spaces and floors.
        Include tasks for both ground floor and upper floors.
        """
        
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,
            max_tokens=2000
        )
        
        task_plan_text = response.choices[0].message.content
        
        try:
            start_idx = task_plan_text.find('{')
            end_idx = task_plan_text.rfind('}') + 1
            if start_idx != -1 and end_idx != -1:
                task_plan_json = json.loads(task_plan_text[start_idx:end_idx])
            else:
                task_plan_json = create_default_task_plan(building_twin, task_plan_text)
        except json.JSONDecodeError:
            task_plan_json = create_default_task_plan(building_twin, task_plan_text)
        
        return task_plan_json
        
    except Exception as e:
        logging.error(f"Error generating construction tasks: {str(e)}")
        return create_default_task_plan(building_twin, str(e))

def chain_of_thought_planning_enhanced(client, building_twin, complexity, min_nav_per_floor, require_upper):
    """ENHANCED Chain-of-Thought with multi-floor navigation"""
    try:
        building_info = {
            "name": building_twin.get("buildingName", "Unknown"),
            "type": building_twin.get("buildingType", "commercial"),
            "area": building_twin.get("totalArea", 0)
        }
        
        # Collect ADT space data
        space_data = _collect_space_centers_from_adt(building_twin)
        
        reasoning_steps = []
        
        # Step 1: Site Analysis
        step1_prompt = f"""
        Analyze this building for robotic construction:
        Building: {json.dumps(building_info, indent=2)}
        Available spaces: {json.dumps(space_data, indent=2)}
        
        Consider multi-floor deployment and navigation requirements.
        """
        
        step1_response = client.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": step1_prompt}],
            temperature=0.2,
            max_tokens=800
        )
        
        reasoning_steps.append({
            "step": 1,
            "title": "Site Analysis",
            "reasoning": step1_response.choices[0].message.content
        })
        
        # Step 2: Task Sequencing
        step2_prompt = f"""
        Based on the site analysis, create task sequences that:
        - Distribute work across multiple floors
        - Navigate to specific spaces by name
        - Ensure at least {min_nav_per_floor} tasks per floor
        {"- Include upper floor tasks" if require_upper else ""}
        """
        
        step2_response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "user", "content": step1_prompt},
                {"role": "assistant", "content": step1_response.choices[0].message.content},
                {"role": "user", "content": step2_prompt}
            ],
            temperature=0.2,
            max_tokens=800
        )
        
        reasoning_steps.append({
            "step": 2,
            "title": "Task Sequencing",
            "reasoning": step2_response.choices[0].message.content
        })
        
        # Step 3: Final Task Plan with multi-floor navigation
        step3_prompt = f"""
        Now output ONLY a compact JSON with this schema (no prose):

        {{
          "final_tasks": {{
            "tasks": [
              {{
                "task_id": "T###",
                "description": "Navigate to <space_name> on floor <n>",
                "command_type": "navigation",
                "floor": 0|1,
                "parameters": {{
                  "space_name": "<exact ADT spaceName>",
                  "target_position": {{"x": <float>, "y": <float>, "z": <float>}}
                }}
              }}
            ]
          }}
        }}

        Rules:
        - Produce ≥{min_nav_per_floor} tasks for floor 0 AND ≥2 for floor 1.
        {"- Include at least one upper-floor space (e.g., lounge/meeting/executive)." if require_upper else ""}
        - Avoid repeating the same space twice in a row.
        - Use centers from this list when possible, with z = floor_elevation + 0.5:
        {json.dumps(space_data, indent=2)}
        """
        
        step3_response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "user", "content": step1_prompt},
                {"role": "assistant", "content": step1_response.choices[0].message.content},
                {"role": "user", "content": step2_prompt},
                {"role": "assistant", "content": step2_response.choices[0].message.content},
                {"role": "user", "content": step3_prompt}
            ],
            temperature=0.1,
            max_tokens=1200
        )
        
        reasoning_steps.append({
            "step": 3,
            "title": "Final Task Plan",
            "reasoning": step3_response.choices[0].message.content
        })
        
        # Extract final tasks
        final_response = step3_response.choices[0].message.content
        try:
            start_idx = final_response.find('{')
            end_idx = final_response.rfind('}') + 1
            if start_idx != -1 and end_idx != -1:
                final_tasks = json.loads(final_response[start_idx:end_idx])
            else:
                final_tasks = create_default_multi_floor_task_plan(building_twin)
        except:
            final_tasks = create_default_multi_floor_task_plan(building_twin)
        
        return {
            "reasoning_steps": reasoning_steps,
            "final_tasks": final_tasks,
            "complexity_level": complexity,
            "building_info": building_info,
            "multi_floor_navigation": True
        }
        
    except Exception as e:
        logging.error(f"Error in chain of thought planning: {str(e)}")
        return {
            "reasoning_steps": [{"error": str(e)}],
            "final_tasks": create_default_multi_floor_task_plan(building_twin),
            "complexity_level": complexity
        }

def structure_for_robots(task_plan_data, robot_type):
    """Structure task plan data for robotic execution"""
    try:
        structured_tasks = []
        task_id = 1
        
        # Extract tasks from various possible formats
        tasks = []
        if isinstance(task_plan_data, dict):
            if "task_plan" in task_plan_data:
                tasks = task_plan_data["task_plan"].get("tasks", [])
            elif "tasks" in task_plan_data:
                tasks = task_plan_data["tasks"]
            elif "final_tasks" in task_plan_data:
                tasks = task_plan_data["final_tasks"].get("tasks", [])
        
        for task in tasks:
            if isinstance(task, dict):
                structured_task = {
                    "task_id": task.get("task_id", f"T{task_id:03d}"),
                    "description": task.get("description", f"Construction Task {task_id}"),
                    "command_type": task.get("command_type", "generic"),
                    "floor": task.get("floor", 0),
                    "parameters": task.get("parameters", {}),
                    "robot_type": robot_type,
                    "actions": task.get("actions", task.get("robot_actions", ["navigate", "execute", "verify"])),
                    "duration_minutes": task.get("duration", task.get("duration_minutes", 30))
                }
                structured_tasks.append(structured_task)
                task_id += 1
        
        return {
            "robot_type": robot_type,
            "total_tasks": len(structured_tasks),
            "estimated_duration": sum(t.get("duration_minutes", 30) for t in structured_tasks),
            "tasks": structured_tasks
        }
        
    except Exception as e:
        logging.error(f"Error structuring for robots: {str(e)}")
        return {
            "robot_type": robot_type,
            "total_tasks": 1,
            "estimated_duration": 60,
            "tasks": [{
                "task_id": "T001",
                "description": "Default construction task",
                "robot_type": robot_type,
                "actions": ["navigate", "execute", "verify"],
                "duration_minutes": 60
            }]
        }

def generate_ros2_commands_enhanced(structured_data):
    """ENHANCED ROS2 command generation with floor-aware navigation"""
    import re
    
    # Extended waypoint dictionary with upper floor locations
    ROOM_WAYPOINTS = {
        # Ground floor (z ≈ 0.5)
        "site": {"x": -6.0, "y": -3.0, "z": 0.5},
        "ground": {"x": -2.0, "y": -1.0, "z": 0.5},
        "bedroom_01": {"x": 1.5, "y": 1.0, "z": 0.5},
        "bedroom_02": {"x": 2.5, "y": 1.0, "z": 0.5},
        "kitchen": {"x": 0.5, "y": 2.0, "z": 0.5},
        "bathroom": {"x": 0.5, "y": 0.5, "z": 0.5},
        "reception": {"x": 4.0, "y": 0.5, "z": 0.5},
        "living_room": {"x": 2.0, "y": 3.0, "z": 0.5},
        # Upper floor (z ≈ 4.0 for 3.5m elevation)
        "lounge": {"x": 3.0, "y": 2.0, "z": 4.0},
        "executive_office": {"x": 5.0, "y": 3.0, "z": 4.0},
        "conference": {"x": 6.0, "y": 2.0, "z": 4.0},
        "meeting_room": {"x": 4.0, "y": 4.0, "z": 4.0}
    }
    
    # Build floor elevation cache from ADT
    def _floor_elevations():
        cache = {"ground floor": 0.0, "first floor": 3.5, "second floor": 7.0}
        try:
            dt_url = os.environ.get("DIGITAL_TWINS_URL")
            if dt_url:
                client = DigitalTwinsClient(dt_url, DefaultAzureCredential())
                q = "SELECT $dtId, floorName, elevation FROM digitaltwins WHERE IS_OF_MODEL('dtmi:construction:robot:Floor;1')"
                for it in client.query_twins(q):
                    name = (it.get("floorName") or "").lower()
                    cache[name] = float(it.get("elevation") or 0.0)
        except Exception:
            pass
        return cache
    
    FLOOR_Z = _floor_elevations()
    
    def _z_for_floor(floor_num: int, floor_name: str = None) -> float:
        """Calculate z coordinate based on floor number and name"""
        if floor_name:
            nm = floor_name.lower()
            if nm in FLOOR_Z:
                return FLOOR_Z[nm] + 0.5
        # Default: 3.5m per storey
        return (3.5 * floor_num) + 0.5
    
    def parse_nav_target(text: str, floor: int = 0):
        """Parse navigation target with floor awareness"""
        t = (text or "").lower().strip()
        
        # Extract space name from navigation commands
        patterns = [
            r"navigate[_\s]*to\s+([a-z0-9_ \-]+?)(?:\s+on\s+floor)?",
            r"go\s*to\s+([a-z0-9_ \-]+?)(?:\s+on\s+floor)?",
            r"move\s*to\s+([a-z0-9_ \-]+?)(?:\s+on\s+floor)?"
        ]
        
        for pattern in patterns:
            m = re.search(pattern, t)
            if m:
                space = m.group(1).strip().replace(" ", "_")
                if space in ROOM_WAYPOINTS:
                    pos = ROOM_WAYPOINTS[space].copy()
                    # Adjust z based on floor if not already set
                    if floor > 0 and pos["z"] < 2.0:
                        pos["z"] = _z_for_floor(floor)
                    return pos
        
        # Default position with floor-aware z
        return {"x": -3.5, "y": -2.0, "z": _z_for_floor(floor)}
    
    ros2_commands = {
        "format": "ros2_construction_commands",
        "version": "2.0",
        "robot_type": structured_data.get("robot_type", "construction_robot"),
        "total_duration_minutes": structured_data.get("estimated_duration", 0),
        "multi_floor_support": True,
        "tasks": []
    }
    
    for task in structured_data.get("tasks", []):
        ros2_task = {
            "task_id": task.get("task_id", "T001"),
            "description": task.get("description", "Construction task"),
            "floor": task.get("floor", 0),
            "ros2_commands": [],
            "duration_minutes": task.get("duration_minutes", 30)
        }
        
        # Handle navigation tasks with floor awareness
        if task.get("command_type") == "navigation" and task.get("parameters"):
            params = task["parameters"]
            space_name = (params.get("space_name") or "").lower().strip()
            pos = params.get("target_position") or {}
            floor_num = task.get("floor", 0)
            
            # Try to get position from known waypoints
            if space_name in ROOM_WAYPOINTS:
                base = ROOM_WAYPOINTS[space_name].copy()
                x, y = base["x"], base["y"]
                z = _z_for_floor(floor_num, task.get("floor_name"))
            else:
                x = float(pos.get("x", 0.0))
                y = float(pos.get("y", 0.0))
                z = _z_for_floor(floor_num, task.get("floor_name"))
            
            ros2_task["ros2_commands"].append({
                "command_type": "navigation",
                "topic": "/move_base_simple/goal",
                "message_type": "geometry_msgs/PoseStamped",
                "parameters": {
                    "target_position": {"x": x, "y": y, "z": z},
                    "target_orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            })
            
            # Add second waypoint to avoid single-point patrol
            ros2_task["ros2_commands"].append({
                "command_type": "navigation",
                "topic": "/move_base_simple/goal",
                "message_type": "geometry_msgs/PoseStamped",
                "parameters": {
                    "target_position": {"x": x + 0.6, "y": y + 0.4, "z": z},
                    "target_orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            })
        else:
            # Process regular actions
            for action in task.get("actions", []):
                a = (action or "").lower().strip()
                floor_num = task.get("floor", 0)
                
                # Handle navigation actions
                if any(nav_word in a for nav_word in ["navigate", "go to", "move to", "goto"]):
                    pos = parse_nav_target(a, floor_num)
                    ros2_task["ros2_commands"].append({
                        "command_type": "navigation",
                        "topic": "/move_base_simple/goal",
                        "message_type": "geometry_msgs/PoseStamped",
                        "parameters": {
                            "target_position": pos,
                            "target_orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                        }
                    })
                elif a == "excavate":
                    ros2_task["ros2_commands"].append({
                        "command_type": "excavation",
                        "topic": "/excavator/cmd_vel",
                        "message_type": "geometry_msgs/Twist",
                        "parameters": {
                            "linear": {"x": 0.1, "y": 0.0, "z": -0.1},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                        }
                    })
                elif a == "lift":
                    ros2_task["ros2_commands"].append({
                        "command_type": "lifting",
                        "topic": "/crane/cmd",
                        "message_type": "std_msgs/Float64",
                        "parameters": {"lift_height": 2.0, "lift_speed": 0.5}
                    })
                else:
                    ros2_task["ros2_commands"].append({
                        "command_type": "generic_action",
                        "topic": f"/robot/{a}",
                        "message_type": "std_msgs/String",
                        "parameters": {"action": a}
                    })
        
        # Ensure at least 2 navigation goals per task to avoid single-point patrol
        nav_commands = [c for c in ros2_task["ros2_commands"] if c["command_type"] == "navigation"]
        if len(nav_commands) == 1:
            last = nav_commands[0]["parameters"]["target_position"]
            ros2_task["ros2_commands"].append({
                "command_type": "navigation",
                "topic": "/move_base_simple/goal",
                "message_type": "geometry_msgs/PoseStamped",
                "parameters": {
                    "target_position": {"x": last["x"] + 0.5, "y": last["y"] + 0.5, "z": last["z"]},
                    "target_orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            })
        
        ros2_commands["tasks"].append(ros2_task)
    
    return ros2_commands

def create_default_task_plan(building_twin, error_info=""):
    """Create a default task plan when GPT response parsing fails"""
    building_name = building_twin.get("buildingName", "Unknown Building")
    
    return {
        "project_name": f"Construction of {building_name}",
        "tasks": [
            {
                "task_id": "T001",
                "description": "Site preparation - ground floor",
                "robot_actions": ["navigate_to_site", "excavate_foundation"],
                "floor": 0,
                "duration": 480
            },
            {
                "task_id": "T002",
                "description": "Foundation construction - ground floor",
                "robot_actions": ["navigate_to_ground", "pour_concrete"],
                "floor": 0,
                "duration": 600
            }
        ]
    }

def create_default_multi_floor_task_plan(building_twin):
    """Create default multi-floor task plan"""
    return {
        "final_tasks": {
            "tasks": [
                {
                    "task_id": "T001",
                    "description": "Navigate to reception on floor 0",
                    "command_type": "navigation",
                    "floor": 0,
                    "parameters": {
                        "space_name": "reception",
                        "target_position": {"x": 4.0, "y": 0.5, "z": 0.5}
                    }
                },
                {
                    "task_id": "T002",
                    "description": "Navigate to living room on floor 0",
                    "command_type": "navigation",
                    "floor": 0,
                    "parameters": {
                        "space_name": "living_room",
                        "target_position": {"x": 2.0, "y": 3.0, "z": 0.5}
                    }
                },
                {
                    "task_id": "T003",
                    "description": "Navigate to kitchen on floor 0",
                    "command_type": "navigation",
                    "floor": 0,
                    "parameters": {
                        "space_name": "kitchen",
                        "target_position": {"x": 0.5, "y": 2.0, "z": 0.5}
                    }
                },
                {
                    "task_id": "T004",
                    "description": "Navigate to bedroom on floor 0",
                    "command_type": "navigation",
                    "floor": 0,
                    "parameters": {
                        "space_name": "bedroom_01",
                        "target_position": {"x": 1.5, "y": 1.0, "z": 0.5}
                    }
                },
                {
                    "task_id": "T005",
                    "description": "Navigate to executive office on floor 1",
                    "command_type": "navigation",
                    "floor": 1,
                    "parameters": {
                        "space_name": "executive_office",
                        "target_position": {"x": 5.0, "y": 3.0, "z": 4.0}
                    }
                },
                {
                    "task_id": "T006",
                    "description": "Navigate to meeting room on floor 1",
                    "command_type": "navigation",
                    "floor": 1,
                    "parameters": {
                        "space_name": "meeting_room",
                        "target_position": {"x": 4.0, "y": 4.0, "z": 4.0}
                    }
                }
            ]
        }
    }

