import json
import math
import os
import sys
from typing import List, Tuple, Dict, Any

class Die:
    def __init__(self, corners: List[List[float]]):
        self.corners = corners
        self.center = self.calculate_center()
        self.visited = False
        
    def calculate_center(self) -> Tuple[float, float]:
        """Calculate the center of the die from its corners (average of min/max)."""
        x_coords = [corner[0] for corner in self.corners]
        y_coords = [corner[1] for corner in self.corners]
        
        center_x = (min(x_coords) + max(x_coords)) / 2
        center_y = (min(y_coords) + max(y_coords)) / 2
        
        return (center_x, center_y)

class WaferInspector:
    def __init__(self, input_data: Dict[str, Any]):
        self.initial_position = tuple(input_data.get("InitialPosition", [0, 0]))
        self.initial_angle = input_data.get("InitialAngle", 0)
        self.stage_velocity = input_data.get("StageVelocity", 50)
        self.stage_acceleration = input_data.get("StageAcceleration", 0)
        self.camera_velocity = input_data.get("CameraVelocity", 5)
        self.camera_acceleration = input_data.get("CameraAcceleration", 0)
        self.wafer_diameter = input_data.get("WaferDiameter", 300)
        
        # Create Die objects from input
        self.dies = [Die(die_data["Corners"]) for die_data in input_data.get("Dies", [])]
        
    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def find_nearest_neighbor_path(self) -> Tuple[List[Tuple[float, float]], float]:
        """Find path using nearest neighbor algorithm for non-linear arrangements."""
        if not self.dies:
            return [self.initial_position], 0.0
        
        # Reset visited flags
        for die in self.dies:
            die.visited = False
        
        path = [self.initial_position]
        total_distance = 0.0
        current_position = self.initial_position
        remaining_dies = len(self.dies)
        
        while remaining_dies > 0:
            # Find nearest unvisited die
            nearest_idx = -1
            min_dist = float('inf')
            
            for i, die in enumerate(self.dies):
                if not die.visited:
                    dist = self.calculate_distance(current_position, die.center)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_idx = i
            
            if nearest_idx == -1:
                break
                
            # Move to nearest die
            current_position = self.dies[nearest_idx].center
            path.append(current_position)
            total_distance += min_dist
            
            # Mark as visited
            self.dies[nearest_idx].visited = True
            remaining_dies -= 1
        
        total_time = total_distance / self.stage_velocity
        return path, total_time
    
    def find_linear_traversal_path(self) -> Tuple[List[Tuple[float, float]], float]:
        """Find path using linear traversal for linearly arranged dies."""
        if not self.dies:
            return [self.initial_position], 0.0
        
        # Get all die centers
        die_centers = [die.center for die in self.dies]
        
        # Determine primary alignment: horizontal (sort by x) or vertical (sort by y)
        x_coords = [c[0] for c in die_centers]
        y_coords = [c[1] for c in die_centers]
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        
        if x_range >= y_range:
            # Horizontal alignment - sort primarily by x, then by y for stability
            sorted_centers = sorted(die_centers, key=lambda c: (c[0], c[1]))
        else:
            # Vertical alignment - sort primarily by y, then by x
            sorted_centers = sorted(die_centers, key=lambda c: (c[1], c[0]))
        forward_distance = self.calculate_distance(self.initial_position, sorted_centers[0])
        for i in range(len(sorted_centers) - 1):
            forward_distance += self.calculate_distance(sorted_centers[i], sorted_centers[i + 1])
        reverse_distance = self.calculate_distance(self.initial_position, sorted_centers[-1])
        for i in range(len(sorted_centers) - 1, 0, -1):
            reverse_distance += self.calculate_distance(sorted_centers[i], sorted_centers[i - 1])
        if forward_distance <= reverse_distance:
            path = [self.initial_position] + sorted_centers
            total_distance = forward_distance
        else:
            path = [self.initial_position] + list(reversed(sorted_centers))
            total_distance = reverse_distance
        
        total_time = total_distance / self.stage_velocity
        return path, total_time
    
    def find_optimal_path(self) -> Tuple[List[Tuple[float, float]], float]:
        """
        Find optimal path by trying both approaches and choosing the best.
        Automatically detects if dies are linear or scattered.
        """
        if not self.dies:
            return [self.initial_position], 0.0
        linear_path, linear_time = self.find_linear_traversal_path()
        
        # Reset visited flags for nearest neighbor
        for die in self.dies:
            die.visited = False
        nn_path, nn_time = self.find_nearest_neighbor_path()
        
        # Choose the better path based on time
        if linear_time <= nn_time:
            return linear_path, linear_time
        else:
            return nn_path, nn_time
    
    def run_inspection(self) -> Dict[str, Any]:
        """Run the inspection and return results in the required format."""
        path, total_time = self.find_optimal_path()
        
        # Format path as list of lists with floats for JSON
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
    output_filename = filename.replace('.json', '_output.json')
    with open(output_filename, 'w') as file:
        json.dump(result, file, indent=2)
    print(f"Result saved to {output_filename}")

def display_menu():
    """Display the main menu."""
    print("\n" + "="*50)
    print("       Wafer Inspection Path Optimizer")
    print("="*50)
    print("1. Run single test case")
    print("2. Run all test cases")
    print("3. Exit")
    print("="*50)

def run_single_test_case():
    """Run a single test case specified by the user."""
    filename = input("Enter the test case filename (e.g., Input_Milestone1_Testcase1.json): ").strip()
    
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' does not exist.")
        return
    
    print(f"\nLoading test case: {filename}")
    input_data = load_test_case(filename)
    
    if not input_data:
        print("Failed to load test case.")
        return
    
    inspector = WaferInspector(input_data)
    result = inspector.run_inspection()
    
    print("\nResults:")
    print(f"Total Time: {result['TotalTime']} seconds")
    print(f"Path length: {len(result['Path'])} points")
    print(f"Expected: {len(inspector.dies) + 1} points (initial + {len(inspector.dies)} dies)")
    
    # Verify all dies were visited
    if len(result['Path']) != len(inspector.dies) + 1:
        print(f"WARNING: Expected {len(inspector.dies) + 1} points, got {len(result['Path'])}")
    
    # Show path details
    print("\nPath:")
    for i, point in enumerate(result['Path']):
        print(f"  {i}: [{point[0]:.2f}, {point[1]:.2f}]")
    
    # Save result
    save_choice = input("\nSave result to file? (y/n): ").strip().lower()
    if save_choice == 'y':
        save_result(filename, result)

def run_all_test_cases():
    """Run all test cases in the current directory."""
    # Look for all test case files
    test_files = []
    for file in os.listdir('.'):
        if file.startswith('Input_') and file.endswith('.json'):
            test_files.append(file)
    
    if not test_files:
        print("No test case files found in current directory.")
        print("Looking for files starting with 'Input_' and ending with '.json'")
        return
    
    print(f"\nFound {len(test_files)} test case(s):")
    for tc in sorted(test_files):
        print(f"  - {tc}")
    
    print("\n" + "="*50)
    print("Processing all test cases...")
    print("="*50)
    
    all_passed = True
    
    for filename in sorted(test_files):
        print(f"\nProcessing: {filename}")
        
        input_data = load_test_case(filename)
        if not input_data:
            print(f"  Skipping {filename} due to load error")
            all_passed = False
            continue
        
        inspector = WaferInspector(input_data)
        result = inspector.run_inspection()
        
        # Verify all dies were visited
        expected_points = len(inspector.dies) + 1
        actual_points = len(result['Path'])
        
        if actual_points != expected_points:
            print(f"  FAILED: Path has {actual_points} points, expected {expected_points}")
            all_passed = False
        else:
            print(f"  PASSED: All {len(inspector.dies)} dies visited")
        
        save_result(filename, result)
        
        print(f"  Total Time: {result['TotalTime']} seconds")
        print(f"  Path points: {actual_points}")
    
    print("\n" + "="*50)
    if all_passed:
        print("ALL TEST CASES PASSED ✓")
    else:
        print("SOME TEST CASES FAILED ✗")
    print("="*50)

def main():
    """Main function with menu-driven interface."""
    print("Wafer Inspection Path Planner")
    print("Algorithm: Hybrid (Linear Traversal + Nearest Neighbor)")
    print("Automatically chooses best approach for each test case")
    
    while True:
        display_menu()
        
        try:
            choice = input("\nEnter your choice (1-3): ").strip()
            
            if choice == '1':
                run_single_test_case()
            elif choice == '2':
                run_all_test_cases()
            elif choice == '3':
                print("\nExiting program...")
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
                
            input("\nPress Enter to continue...")
            
        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Exiting...")
            break
        except Exception as e:
            print(f"\nAn error occurred: {e}")
            import traceback
            traceback.print_exc()
            input("\nPress Enter to continue...")

if __name__ == "__main__":
    # Support command-line execution: python script.py <input.json>
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        if os.path.exists(filename):
            input_data = load_test_case(filename)
            if input_data:
                inspector = WaferInspector(input_data)
                result = inspector.run_inspection()
                print(json.dumps(result, indent=2))
            else:
                print(f"Failed to load {filename}")
        else:
            print(f"File {filename} not found")
    else:
        # Interactive mode
        test_files = [f for f in os.listdir('.') if f.startswith('Input_') and f.endswith('.json')]
        
        if test_files:
            print(f"Found {len(test_files)} test case file(s):")
            for f in sorted(test_files):
                print(f"  - {f}")
            print()
            
            auto_run = input("Run all test cases automatically? (y/n): ").strip().lower()
            if auto_run == 'y':
                run_all_test_cases()
                sys.exit(0)
        
        main()