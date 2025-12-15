import json
import math
import os
import sys
from typing import List, Tuple, Dict, Any

class Die:
    def __init__(self, corners: List[List[float]], milestone: int = 1):
        self.corners = corners
        self.center = self.calculate_center()
        self.milestone = milestone
        if milestone == 2:
            self.orientation = self.calculate_orientation()
        self.visited = False
    def calculate_center(self) -> Tuple[float, float]:
        x_coords = [corner[0] for corner in self.corners]
        y_coords = [corner[1] for corner in self.corners]
        center_x = (min(x_coords) + max(x_coords)) / 2
        center_y = (min(y_coords) + max(y_coords)) / 2
        return (center_x, center_y)
    def calculate_orientation(self) -> float:
        """Calculate the orientation angle of the die in degrees (0-360)."""
        if len(self.corners) < 2:
            return 0.0
        x1, y1 = self.corners[0]
        x2, y2 = self.corners[1]
        
        dx = x2 - x1
        dy = y2 - y1
        # Calculate angle in radians, then convert to degrees
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        angle_deg = angle_deg % 360
        return angle_deg
class WaferInspector:
    def __init__(self, input_data: Dict[str, Any], milestone: int = 1):
        self.milestone = milestone
        self.initial_position = tuple(input_data.get("InitialPosition", [0, 0]))
        if milestone == 2:
            self.initial_angle = input_data.get("InitialAngle", 0)
        self.stage_velocity = input_data.get("StageVelocity", 50)
        self.stage_acceleration = input_data.get("StageAcceleration", 0)
        if milestone == 2:
            self.camera_velocity = input_data.get("CameraVelocity", 5)
            self.camera_acceleration = input_data.get("CameraAcceleration", 0)
            self.rotation_velocity = self.camera_velocity 
        self.wafer_diameter = input_data.get("WaferDiameter", 300)
        self.dies = [Die(die_data["Corners"], milestone) 
                    for die_data in input_data.get("Dies", [])]
    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    def calculate_rotation_time(self, camera_angle: float, die_orientation: float) -> float:
        if self.milestone != 2:
            return 0.0
        angle_diff = abs(die_orientation - camera_angle)
        angle_diff = angle_diff % 360
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        if angle_diff < 90:
            return 0.0
        rotation_needed = angle_diff - 90
        # Rotation time = angle / rotation velocity
        return rotation_needed / self.rotation_velocity if self.rotation_velocity > 0 else 0
    def get_new_camera_angle(self, current_angle: float, die_orientation: float) -> float:
        if self.milestone != 2:
            return current_angle
        angle_diff = abs(die_orientation - current_angle)
        angle_diff = angle_diff % 360
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        if angle_diff < 90:
            return current_angle
        raw_diff = (die_orientation - current_angle) % 360
        if raw_diff > 180:
            new_angle = (current_angle - (angle_diff - 90)) % 360
        else:
            new_angle = (current_angle + (angle_diff - 90)) % 360
        return new_angle
    def find_nearest_neighbor_path(self) -> Tuple[List[Tuple[float, float]], float]:
        if not self.dies:
            return [self.initial_position], 0.0
        for die in self.dies:
            die.visited = False
        path = [self.initial_position]
        total_time = 0.0
        current_position = self.initial_position
        if self.milestone == 2:
            current_angle = self.initial_angle
        remaining_dies = len(self.dies)
        while remaining_dies > 0:
            # Find nearest unvisited die
            nearest_idx = -1
            min_total_time = float('inf')
            for i, die in enumerate(self.dies):
                if not die.visited:
                    # Calculate total time to visit this die
                    distance = self.calculate_distance(current_position, die.center)
                    movement_time = distance / self.stage_velocity
                    if self.milestone == 2:
                        rotation_time = self.calculate_rotation_time(current_angle, die.orientation)
                        total_die_time = movement_time + rotation_time
                    else:
                        total_die_time = movement_time
                    
                    if total_die_time < min_total_time:
                        min_total_time = total_die_time
                        nearest_idx = i
            if nearest_idx == -1:
                break
            die = self.dies[nearest_idx]
            distance = self.calculate_distance(current_position, die.center)
            movement_time = distance / self.stage_velocity
            if self.milestone == 2:
                rotation_time = self.calculate_rotation_time(current_angle, die.orientation)
                total_die_time = movement_time + rotation_time
                # Update camera angle
                current_angle = self.get_new_camera_angle(current_angle, die.orientation)
            else:
                total_die_time = movement_time
            # Update position and time
            current_position = die.center
            path.append(current_position)
            total_time += total_die_time
            # Mark as visited
            die.visited = True
            remaining_dies -= 1
        return path, total_time
    def find_linear_traversal_path(self) -> Tuple[List[Tuple[float, float]], float]:
        if not self.dies:
            return [self.initial_position], 0.0
        # Get die information
        die_info = []
        for die in self.dies:
            if self.milestone == 2:
                die_info.append((die.center, die.orientation))
            else:
                die_info.append((die.center, None))
        x_coords = [c[0] for c, _ in die_info]
        y_coords = [c[1] for c, _ in die_info]
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        if x_range >= y_range:
            sorted_info = sorted(die_info, key=lambda item: (item[0][0], item[0][1]))
        else:
            sorted_info = sorted(die_info, key=lambda item: (item[0][1], item[0][0]))
        current_pos = self.initial_position
        forward_time = 0.0
        if self.milestone == 2:
            current_angle = self.initial_angle
        for center, orientation in sorted_info:
            distance = self.calculate_distance(current_pos, center)
            movement_time = distance / self.stage_velocity
            if self.milestone == 2 and orientation is not None:
                rotation_time = self.calculate_rotation_time(current_angle, orientation)
                forward_time += movement_time + rotation_time
                current_angle = self.get_new_camera_angle(current_angle, orientation)
            else:
                forward_time += movement_time
            current_pos = center
        current_pos = self.initial_position
        reverse_time = 0.0
        if self.milestone == 2:
            current_angle = self.initial_angle
        for center, orientation in reversed(sorted_info):
            distance = self.calculate_distance(current_pos, center)
            movement_time = distance / self.stage_velocity
            if self.milestone == 2 and orientation is not None:
                rotation_time = self.calculate_rotation_time(current_angle, orientation)
                reverse_time += movement_time + rotation_time
                current_angle = self.get_new_camera_angle(current_angle, orientation)
            else:
                reverse_time += movement_time
            current_pos = center
        # Choose better path
        if forward_time <= reverse_time:
            total_time = forward_time
            path = [self.initial_position] + [center for center, _ in sorted_info]
        else:
            total_time = reverse_time
            path = [self.initial_position] + [center for center, _ in reversed(sorted_info)]
        
        return path, total_time
    def find_optimal_path(self) -> Tuple[List[Tuple[float, float]], float]:
        """Find optimal path by comparing both approaches."""
        if not self.dies:
            return [self.initial_position], 0.0
        linear_path, linear_time = self.find_linear_traversal_path()
        # Reset for nearest neighbor
        for die in self.dies:
            die.visited = False
        # Try nearest neighbor
        nn_path, nn_time = self.find_nearest_neighbor_path()
        # Choose better path
        if linear_time <= nn_time:
            return linear_path, linear_time
        else:
            return nn_path, nn_time
    def run_inspection(self) -> Dict[str, Any]:
        """Run the inspection and return results."""
        path, total_time = self.find_optimal_path()
        
        formatted_path = [[float(x), float(y)] for x, y in path]
        
        return {
            "TotalTime": round(total_time, 3),
            "Path": formatted_path
        }

def load_test_case(filename: str) -> Dict[str, Any]:
    """Load a test case from a JSON file."""
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return {}
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON in '{filename}'.")
        return {}

def save_result(filename: str, result: Dict[str, Any]):
    """Save the result to a JSON file."""
    # Create output directory if it doesn't exist
    output_dir = "output"
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract base filename without path
    base_name = os.path.basename(filename)
    output_filename = base_name.replace('.json', '_output.json')
    output_path = os.path.join(output_dir, output_filename)
    
    with open(output_path, 'w') as file:
        json.dump(result, file, indent=2)
    
    print(f"Result saved to: {output_path}")
    return output_path

def get_test_cases(milestone: int) -> List[str]:
    """Get all test cases for a specific milestone."""
    milestone_dir = f"Milestone{milestone}"
    
    if not os.path.exists(milestone_dir):
        print(f"Directory '{milestone_dir}' not found.")
        return []
    
    test_files = []
    for file in os.listdir(milestone_dir):
        if file.startswith('Input_') and file.endswith('.json'):
            test_files.append(os.path.join(milestone_dir, file))
    
    return sorted(test_files)

def display_milestone_menu():
    """Display milestone selection menu."""
    print("\n" + "="*60)
    print("="*60)
    print("Select Milestone:")
    print("1. Milestone 1 ")
    print("2. Milestone 2 ")
    print("3. Exit")
    print("="*60)

def display_test_case_menu(milestone: int, test_cases: List[str]):
    """Display test case selection menu."""
    print(f"\n{'='*60}")
    print(f"Milestone {milestone} - Available Test Cases")
    print(f"{'='*60}")
    
    for i, test_case in enumerate(test_cases, 1):
        base_name = os.path.basename(test_case)
        print(f"{i}. {base_name}")
    
    print(f"{len(test_cases) + 1}. Run all test cases")
    print(f"{len(test_cases) + 2}. Back to milestone selection")
    print(f"{'='*60}")

def run_single_test_case(milestone: int, filename: str):
    """Run a single test case."""
    print(f"\n{'='*60}")
    print(f"Running: {os.path.basename(filename)}")
    print(f"Milestone: {milestone}")
    print(f"{'='*60}")
    
    # Load test case
    input_data = load_test_case(filename)
    
    if not input_data:
        print(f"Failed to load test case: {filename}")
        return
    
    # Create inspector and run inspection
    inspector = WaferInspector(input_data, milestone)
    result = inspector.run_inspection()
    
    # Display results
    print(f"\nResults:")
    print(f"  Total Time: {result['TotalTime']} seconds")
    print(f"  Path Points: {len(result['Path'])}")
    print(f"  Dies Visited: {len(inspector.dies)}")
    
    # Show path preview
    if len(result['Path']) > 0:
        print(f"\nPath Preview (first 3 points):")
        for i, point in enumerate(result['Path'][:3]):
            print(f"  Point {i}: [{point[0]:.2f}, {point[1]:.2f}]")
        if len(result['Path']) > 3:
            print(f"  ... and {len(result['Path']) - 3} more points")
    
    # Save result
    save_choice = input("\nSave result to file? (y/n): ").strip().lower()
    if save_choice == 'y':
        output_path = save_result(filename, result)
        
        # Option to show saved file
        show_choice = input("Show saved file content? (y/n): ").strip().lower()
        if show_choice == 'y':
            with open(output_path, 'r') as f:
                print("\n" + "="*60)
                print("Saved Output:")
                print("="*60)
                print(f.read())
    
    print(f"\n{'='*60}")
    print("Test case completed!")
    print(f"{'='*60}")

def run_all_test_cases(milestone: int, test_cases: List[str]):
    """Run all test cases for a milestone."""
    print(f"\n{'='*60}")
    print(f"Running all test cases for Milestone {milestone}")
    print(f"{'='*60}")
    
    if not test_cases:
        print("No test cases found!")
        return
    
    results_summary = []
    
    for i, filename in enumerate(test_cases, 1):
        print(f"\n[{i}/{len(test_cases)}] Processing: {os.path.basename(filename)}")
        
        try:
            # Load test case
            input_data = load_test_case(filename)
            
            if not input_data:
                print(f"  ✗ Failed to load")
                results_summary.append((os.path.basename(filename), "FAILED", "Load error"))
                continue
            
            # Create inspector and run
            inspector = WaferInspector(input_data, milestone)
            result = inspector.run_inspection()
            
            # Save result
            output_path = save_result(filename, result)
            
            # Verify all dies were visited
            expected_points = len(inspector.dies) + 1
            actual_points = len(result['Path'])
            
            if actual_points == expected_points:
                status = "PASSED"
            else:
                status = "FAILED"
            
            results_summary.append((
                os.path.basename(filename),
                status,
                f"{result['TotalTime']}s, {actual_points} points"
            ))
            
            print(f"  ✓ {status} - Time: {result['TotalTime']}s, Points: {actual_points}")
            
        except Exception as e:
            print(f"  ✗ ERROR: {str(e)}")
            results_summary.append((os.path.basename(filename), "ERROR", str(e)))
    
    # Display summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    
    passed = sum(1 for _, status, _ in results_summary if status == "PASSED")
    failed = sum(1 for _, status, _ in results_summary if status == "FAILED")
    errors = sum(1 for _, status, _ in results_summary if status == "ERROR")
    
    print(f"Total: {len(results_summary)}")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Errors: {errors}")
    
    if failed == 0 and errors == 0:
        print("\n✓ All test cases passed successfully!")
    else:
        print("\nDetailed Results:")
        for filename, status, details in results_summary:
            if status != "PASSED":
                print(f"  {filename}: {status} - {details}")
    
    print(f"\n{'='*60}")

def main():
    """Main interactive menu-driven program."""
    print("="*60)
    print("    WAFER INSPECTION PATH OPTIMIZER")
    print("        Combined Milestone 1 & 2")
    print("="*60)
    
    while True:
        # Step 1: Select milestone
        display_milestone_menu()
        
        try:
            milestone_choice = input("\nEnter your choice (1-3): ").strip()
            
            if milestone_choice == '3':
                print("\nExiting program...")
                break
            
            if milestone_choice not in ['1', '2']:
                print("Invalid choice. Please select 1, 2, or 3.")
                input("\nPress Enter to continue...")
                continue
            
            milestone = int(milestone_choice)
            
            # Get test cases for selected milestone
            test_cases = get_test_cases(milestone)
            
            if not test_cases:
                print(f"\nNo test cases found in Milestone{milestone} folder.")
                print("Please ensure the folder exists with test case files.")
                input("\nPress Enter to continue...")
                continue
            
            # Step 2: Select test case or action
            while True:
                display_test_case_menu(milestone, test_cases)
                
                test_choice = input(f"\nEnter your choice (1-{len(test_cases) + 2}): ").strip()
                
                try:
                    choice_num = int(test_choice)
                    
                    if choice_num == len(test_cases) + 2:
                        # Go back to milestone selection
                        break
                    elif choice_num == len(test_cases) + 1:
                        # Run all test cases
                        run_all_test_cases(milestone, test_cases)
                        input("\nPress Enter to continue...")
                    elif 1 <= choice_num <= len(test_cases):
                        # Run single test case
                        selected_file = test_cases[choice_num - 1]
                        run_single_test_case(milestone, selected_file)
                        input("\nPress Enter to continue...")
                    else:
                        print("Invalid choice. Please try again.")
                
                except ValueError:
                    print("Please enter a valid number.")
            
        except KeyboardInterrupt:
            print("\n\nProgram interrupted by user. Exiting...")
            break
        except Exception as e:
            print(f"\nAn error occurred: {e}")
            import traceback
            traceback.print_exc()
            input("\nPress Enter to continue...")

def run_from_command_line():
    """Run from command line with arguments."""
    if len(sys.argv) != 3:
        print("Usage: python wafer_inspector.py <milestone> <test_case>")
        print("Example: python wafer_inspector.py 1 Milestone1/Input_Testcase1.json")
        print("\nOr run without arguments for interactive mode.")
        return
    
    try:
        milestone = int(sys.argv[1])
        filename = sys.argv[2]
        
        if milestone not in [1, 2]:
            print("Error: Milestone must be 1 or 2")
            return
        
        if not os.path.exists(filename):
            print(f"Error: File '{filename}' not found.")
            return
        
        # Load and run test case
        input_data = load_test_case(filename)
        
        if not input_data:
            print(f"Failed to load test case: {filename}")
            return
        
        inspector = WaferInspector(input_data, milestone)
        result = inspector.run_inspection()
        
        # Display result
        print(json.dumps(result, indent=2))
        
        # Save result automatically
        output_path = save_result(filename, result)
        print(f"\nOutput saved to: {output_path}")
        
    except ValueError:
        print("Error: Milestone must be a number (1 or 2)")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Check if command line arguments are provided
    if len(sys.argv) > 1:
        run_from_command_line()
    else:
        # Run in interactive mode
        main()