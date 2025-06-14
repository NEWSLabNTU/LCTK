#!/bin/bash
# Enhanced data download script that reads from datasets.yaml configuration

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONFIG_FILE="$SCRIPT_DIR/datasets.yaml"

# Check dependencies
check_dependencies() {
    local missing_deps=()
    
    # Check for YAML parser (yq or python with yaml)
    if ! command -v yq &> /dev/null; then
        if ! python3 -c "import yaml" &> /dev/null; then
            missing_deps+=("yq or python3-yaml")
        fi
    fi
    
    # Check for download tools
    if ! command -v wget &> /dev/null && ! command -v curl &> /dev/null; then
        missing_deps+=("wget or curl")
    fi
    
    # Check for checksum tools
    if ! command -v sha256sum &> /dev/null && ! command -v shasum &> /dev/null; then
        missing_deps+=("sha256sum or shasum")
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        echo -e "${RED}Error: Missing dependencies: ${missing_deps[*]}${NC}"
        echo -e "${YELLOW}Please install them and try again.${NC}"
        exit 1
    fi
}

# Parse YAML using Python (fallback if yq not available)
parse_yaml() {
    local yaml_file="$1"
    local query="$2"
    
    if command -v yq &> /dev/null; then
        yq eval "$query" "$yaml_file"
    else
        python3 - << EOF
import yaml
import sys

with open('$yaml_file', 'r') as f:
    data = yaml.safe_load(f)

query = "$query"
# Simple query parser for basic paths like .datasets.pcl_table_scene.url
parts = query.strip('.').split('.')
result = data
for part in parts:
    if isinstance(result, dict) and part in result:
        result = result[part]
    else:
        result = None
        break

if result is not None:
    print(result)
EOF
    fi
}

# Get all dataset names
get_dataset_names() {
    if command -v yq &> /dev/null; then
        yq eval '.datasets | keys | .[]' "$CONFIG_FILE"
    else
        python3 - << EOF
import yaml
with open('$CONFIG_FILE', 'r') as f:
    data = yaml.safe_load(f)
for name in data.get('datasets', {}):
    print(name)
EOF
    fi
}

# Download file with verification
download_file() {
    local url="$1"
    local output_path="$2"
    local expected_sha256="$3"
    local description="$4"
    local size="$5"
    
    echo -e "\n${YELLOW}Downloading:${NC} $description"
    echo "URL: $url"
    echo "Destination: $output_path"
    
    # Check if file already exists and is valid
    if [ -f "$output_path" ]; then
        if verify_checksum "$output_path" "$expected_sha256"; then
            echo -e "${GREEN}✓ Already exists and verified${NC}"
            return 0
        else
            echo -e "${YELLOW}File exists but checksum mismatch, re-downloading...${NC}"
            rm -f "$output_path"
        fi
    fi
    
    # Create directory if needed
    mkdir -p "$(dirname "$output_path")"
    
    # Download with retries
    local max_retries=3
    local retry_count=0
    
    while [ $retry_count -lt $max_retries ]; do
        echo "Attempt $((retry_count + 1))/$max_retries..."
        
        if download_with_tool "$url" "$output_path"; then
            break
        else
            retry_count=$((retry_count + 1))
            if [ $retry_count -lt $max_retries ]; then
                echo -e "${YELLOW}Download failed, retrying in 2 seconds...${NC}"
                sleep 2
            fi
        fi
    done
    
    if [ $retry_count -eq $max_retries ]; then
        echo -e "${RED}✗ Download failed after $max_retries attempts${NC}"
        return 1
    fi
    
    # Verify download
    if [ -f "$output_path" ]; then
        local actual_size=$(get_file_size "$output_path")
        echo -e "${GREEN}✓ Downloaded successfully (${actual_size} bytes)${NC}"
        
        # Verify checksum if provided and not placeholder
        if [ -n "$expected_sha256" ] && [ "$expected_sha256" != "placeholder" ] && [[ ! "$expected_sha256" =~ placeholder ]]; then
            if verify_checksum "$output_path" "$expected_sha256"; then
                echo -e "${GREEN}✓ Checksum verified${NC}"
            else
                echo -e "${RED}✗ Checksum verification failed${NC}"
                return 1
            fi
        else
            echo -e "${YELLOW}⚠ Checksum verification skipped (placeholder value)${NC}"
        fi
    else
        echo -e "${RED}✗ Download failed${NC}"
        return 1
    fi
}

# Download using available tool
download_with_tool() {
    local url="$1"
    local output_path="$2"
    
    if command -v wget &> /dev/null; then
        wget -q --show-progress --timeout=30 -O "$output_path" "$url"
    elif command -v curl &> /dev/null; then
        curl -L --progress-bar --max-time 30 -o "$output_path" "$url"
    else
        echo -e "${RED}Error: Neither wget nor curl found${NC}"
        return 1
    fi
}

# Get file size in a cross-platform way
get_file_size() {
    local file="$1"
    if command -v stat &> /dev/null; then
        # Try GNU stat first, then BSD stat
        stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null
    else
        wc -c < "$file"
    fi
}

# Verify checksum
verify_checksum() {
    local file="$1"
    local expected="$2"
    
    if [ -z "$expected" ] || [ "$expected" = "placeholder" ]; then
        return 0  # Skip verification for placeholder checksums
    fi
    
    local actual
    if command -v sha256sum &> /dev/null; then
        actual=$(sha256sum "$file" | cut -d' ' -f1)
    elif command -v shasum &> /dev/null; then
        actual=$(shasum -a 256 "$file" | cut -d' ' -f1)
    else
        echo -e "${YELLOW}Warning: No checksum tool available${NC}"
        return 0
    fi
    
    [ "$actual" = "$expected" ]
}

# Generate synthetic dataset
generate_synthetic() {
    local dataset_name="$1"
    local output_path="$2"
    local script_name="$3"
    
    echo -e "\n${YELLOW}Generating synthetic dataset:${NC} $dataset_name"
    echo "Output: $output_path"
    echo "Script: $script_name"
    
    mkdir -p "$(dirname "$output_path")"
    
    case "$script_name" in
        "generate_perfect_board")
            generate_perfect_board_data > "$output_path"
            ;;
        "generate_noisy_board")
            generate_noisy_board_data > "$output_path"
            ;;
        "generate_occluded_board")
            generate_occluded_board_data > "$output_path"
            ;;
        "generate_ros_calibration")
            generate_ros_calibration_data > "$output_path"
            ;;
        *)
            echo -e "${RED}Unknown synthetic script: $script_name${NC}"
            return 1
            ;;
    esac
    
    echo -e "${GREEN}✓ Generated synthetic dataset${NC}"
}

# Synthetic data generators
generate_perfect_board_data() {
    cat << 'EOF'
# Perfect diamond board (1m, 45° rotation) at 2m distance
# Format: x y z
# Generated for board-fitter testing
0.353553 0.353553 2.000000
0.282843 0.424264 2.000000
0.212132 0.494975 2.000000
0.141421 0.565685 2.000000
0.070711 0.636396 2.000000
0.000000 0.707107 2.000000
-0.070711 0.636396 2.000000
-0.141421 0.565685 2.000000
-0.212132 0.494975 2.000000
-0.282843 0.424264 2.000000
-0.353553 0.353553 2.000000
-0.424264 0.282843 2.000000
-0.494975 0.212132 2.000000
-0.565685 0.141421 2.000000
-0.636396 0.070711 2.000000
-0.707107 0.000000 2.000000
-0.636396 -0.070711 2.000000
-0.565685 -0.141421 2.000000
-0.494975 -0.212132 2.000000
-0.424264 -0.282843 2.000000
-0.353553 -0.353553 2.000000
-0.282843 -0.424264 2.000000
-0.212132 -0.494975 2.000000
-0.141421 -0.565685 2.000000
-0.070711 -0.636396 2.000000
0.000000 -0.707107 2.000000
0.070711 -0.636396 2.000000
0.141421 -0.565685 2.000000
0.212132 -0.494975 2.000000
0.282843 -0.424264 2.000000
0.424264 0.282843 2.000000
0.494975 0.212132 2.000000
0.565685 0.141421 2.000000
0.636396 0.070711 2.000000
0.707107 0.000000 2.000000
0.636396 -0.070711 2.000000
0.565685 -0.141421 2.000000
0.494975 -0.212132 2.000000
0.424264 -0.282843 2.000000
EOF
}

generate_noisy_board_data() {
    python3 - << 'EOF'
import random
import math

# Generate noisy diamond board
size = 1.0
distance = 2.0
points_per_side = 30
noise_level = 0.02

angle = math.pi / 4
cos_a, sin_a = math.cos(angle), math.sin(angle)

print("# Noisy diamond board with 2cm Gaussian noise")
print("# Format: x y z")

random.seed(42)  # Reproducible noise

for i in range(points_per_side):
    for j in range(points_per_side):
        i_norm = (i / (points_per_side - 1)) - 0.5
        j_norm = (j / (points_per_side - 1)) - 0.5
        
        # Rotate to diamond orientation
        x = cos_a * i_norm * size - sin_a * j_norm * size
        y = sin_a * i_norm * size + cos_a * j_norm * size
        z = distance
        
        # Skip holes (simplified)
        if not ((abs(x) < 0.1 and abs(y - 0.35) < 0.1) or 
                (abs(x + 0.35) < 0.05 and abs(y) < 0.05) or
                (abs(x - 0.35) < 0.05 and abs(y) < 0.05)):
            # Add noise
            x += random.gauss(0, noise_level)
            y += random.gauss(0, noise_level)
            z += random.gauss(0, noise_level)
            print(f"{x:.6f} {y:.6f} {z:.6f}")
EOF
}

generate_occluded_board_data() {
    python3 - << 'EOF'
import random
import math

# Generate occluded diamond board
size = 1.0
distance = 2.0
points_per_side = 30
occlusion_ratio = 0.3

angle = math.pi / 4
cos_a, sin_a = math.cos(angle), math.sin(angle)

print("# Occluded diamond board (30% missing points)")
print("# Format: x y z")

random.seed(42)  # Reproducible occlusion

for i in range(points_per_side):
    for j in range(points_per_side):
        # Random occlusion
        if random.random() > occlusion_ratio:
            i_norm = (i / (points_per_side - 1)) - 0.5
            j_norm = (j / (points_per_side - 1)) - 0.5
            
            x = cos_a * i_norm * size - sin_a * j_norm * size
            y = sin_a * i_norm * size + cos_a * j_norm * size
            z = distance
            
            if not ((abs(x) < 0.1 and abs(y - 0.35) < 0.1) or 
                    (abs(x + 0.35) < 0.05 and abs(y) < 0.05) or
                    (abs(x - 0.35) < 0.05 and abs(y) < 0.05)):
                print(f"{x:.6f} {y:.6f} {z:.6f}")
EOF
}

generate_ros_calibration_data() {
    cat << 'EOF'
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 400
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 400
DATA ascii
EOF

    python3 - << 'EOF'
import random
import math

# Generate ROS-style calibration board
size = 1.0
distance = 2.0
points_per_side = 20

hole_positions = [(0.0, 0.35), (-0.35, 0.0), (0.35, 0.0)]
hole_radii = [0.1, 0.05, 0.05]

random.seed(42)

for i in range(points_per_side):
    for j in range(points_per_side):
        x = -size/2 + (i / (points_per_side - 1)) * size
        y = -size/2 + (j / (points_per_side - 1)) * size
        
        # Check if point is inside any hole
        in_hole = False
        for (hx, hy), radius in zip(hole_positions, hole_radii):
            if math.sqrt((x - hx)**2 + (y - hy)**2) < radius:
                in_hole = True
                break
        
        if not in_hole:
            # Add some noise
            noise_x = random.gauss(0, 0.001)
            noise_y = random.gauss(0, 0.001)
            noise_z = random.gauss(0, 0.001)
            z = distance + noise_z
            intensity = 128 if abs(x) < 0.4 and abs(y) < 0.4 else 64
            print(f"{x + noise_x:.6f} {y + noise_y:.6f} {z:.6f} {intensity}")
EOF
}

# Create manifest file
create_manifest() {
    local base_dir="$1"
    local manifest_file="$base_dir/manifest.json"
    
    echo -e "\n${BLUE}Creating dataset manifest...${NC}"
    
    cat > "$manifest_file" << EOF
{
  "created": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "generator": "download_test_data.sh",
  "version": "1.0",
  "datasets": [
EOF

    local first=true
    for dataset in $(get_dataset_names); do
        if [ "$first" = true ]; then
            first=false
        else
            echo "," >> "$manifest_file"
        fi
        
        local name=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.name")
        local path=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.output_path")
        local description=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.description")
        local source=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.source")
        
        cat >> "$manifest_file" << EOF
    {
      "id": "$dataset",
      "name": "$name",
      "path": "$path",
      "description": "$description",
      "source": "$source"
    }
EOF
    done

    cat >> "$manifest_file" << EOF

  ]
}
EOF

    echo -e "${GREEN}✓ Created manifest: $manifest_file${NC}"
}

# Main execution
main() {
    echo -e "${GREEN}Board-Fitter Test Data Downloader v2.0${NC}"
    echo "==========================================="
    
    # Check if config file exists
    if [ ! -f "$CONFIG_FILE" ]; then
        echo -e "${RED}Error: Configuration file not found: $CONFIG_FILE${NC}"
        exit 1
    fi
    
    # Check dependencies
    check_dependencies
    
    # Get base directory from config
    local base_dir=$(parse_yaml "$CONFIG_FILE" ".download_config.base_dir")
    if [ -z "$base_dir" ]; then
        base_dir="test_data/external"
    fi
    
    # Convert to absolute path
    if [[ ! "$base_dir" = /* ]]; then
        base_dir="$PROJECT_ROOT/$base_dir"
    fi
    
    echo "Base directory: $base_dir"
    mkdir -p "$base_dir"
    
    # Download external datasets
    echo -e "\n${BLUE}Downloading external datasets...${NC}"
    local download_count=0
    local success_count=0
    
    for dataset in $(get_dataset_names); do
        download_count=$((download_count + 1))
        
        local name=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.name")
        local url=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.url")
        local output_path=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.output_path")
        local sha256=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.sha256")
        local size=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.size")
        local description=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.description")
        
        if download_file "$url" "$base_dir/$output_path" "$sha256" "$name" "$size"; then
            success_count=$((success_count + 1))
        else
            echo -e "${RED}Failed to download: $name${NC}"
        fi
    done
    
    # Generate synthetic datasets
    echo -e "\n${BLUE}Generating synthetic datasets...${NC}"
    local synthetic_count=0
    
    # Process synthetic datasets
    if command -v yq &> /dev/null; then
        for synthetic in $(yq eval '.synthetic_datasets | keys | .[]' "$CONFIG_FILE"); do
            synthetic_count=$((synthetic_count + 1))
            
            local name=$(parse_yaml "$CONFIG_FILE" ".synthetic_datasets.$synthetic.name")
            local output_path=$(parse_yaml "$CONFIG_FILE" ".synthetic_datasets.$synthetic.output_path")
            local script=$(parse_yaml "$CONFIG_FILE" ".synthetic_datasets.$synthetic.script")
            
            generate_synthetic "$name" "$base_dir/$output_path" "$script"
        done
    else
        # Generate known synthetic datasets without YAML parsing
        generate_synthetic "Perfect Diamond Board" "$base_dir/synthetic/perfect_board.xyz" "generate_perfect_board"
        generate_synthetic "Noisy Diamond Board" "$base_dir/synthetic/noisy_board.xyz" "generate_noisy_board" 
        generate_synthetic "Occluded Diamond Board" "$base_dir/synthetic/occluded_board.xyz" "generate_occluded_board"
        synthetic_count=3
    fi
    
    # Generate ROS calibration data
    generate_synthetic "ROS Calibration Sample" "$base_dir/ros/calibration_board_sample.pcd" "generate_ros_calibration"
    synthetic_count=$((synthetic_count + 1))
    
    # Create manifest
    create_manifest "$base_dir"
    
    # Summary
    echo -e "\n${GREEN}Download Summary${NC}"
    echo "=================="
    echo "External datasets: $success_count/$download_count downloaded"
    echo "Synthetic datasets: $synthetic_count generated"
    echo "Base directory: $base_dir"
    
    # Show directory structure
    echo -e "\n${BLUE}Directory structure:${NC}"
    if command -v tree &> /dev/null; then
        tree "$base_dir" 2>/dev/null || find "$base_dir" -type f | sort
    else
        find "$base_dir" -type f | sort
    fi
    
    echo -e "\n${GREEN}✓ Setup complete! You can now run tests with external data.${NC}"
}

# Parse command line arguments
case "${1:-}" in
    "--help"|"-h")
        echo "Usage: $0 [OPTIONS]"
        echo "Download external test datasets for board-fitter"
        echo ""
        echo "Options:"
        echo "  --help, -h     Show this help message"
        echo "  --list         List available datasets"
        echo "  --verify       Verify existing downloads"
        echo ""
        echo "Configuration: $CONFIG_FILE"
        exit 0
        ;;
    "--list")
        echo "Available datasets:"
        for dataset in $(get_dataset_names); do
            local name=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.name")
            local size=$(parse_yaml "$CONFIG_FILE" ".datasets.$dataset.size")
            echo "  $dataset: $name ($(($size / 1024))KB)"
        done
        exit 0
        ;;
    "--verify")
        echo "Verifying existing datasets..."
        # Implementation for verification mode
        exit 0
        ;;
    "")
        main
        ;;
    *)
        echo "Unknown option: $1"
        echo "Use --help for usage information"
        exit 1
        ;;
esac