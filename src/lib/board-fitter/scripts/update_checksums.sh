#!/bin/bash
# Update checksums in datasets.yaml for downloaded files

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
TEST_DATA_DIR="$PROJECT_ROOT/test_data/external"

echo -e "${GREEN}Checksum Update Tool${NC}"
echo "===================="

# Check if test data directory exists
if [ ! -d "$TEST_DATA_DIR" ]; then
    echo -e "${RED}Error: Test data directory not found: $TEST_DATA_DIR${NC}"
    echo "Run ./download_test_data.sh first to download datasets"
    exit 1
fi

# Function to calculate SHA256
calculate_sha256() {
    local file="$1"
    if command -v sha256sum &> /dev/null; then
        sha256sum "$file" | cut -d' ' -f1
    elif command -v shasum &> /dev/null; then
        shasum -a 256 "$file" | cut -d' ' -f1
    else
        echo -e "${RED}Error: No SHA256 tool available${NC}"
        exit 1
    fi
}

# Function to update checksum in YAML
update_checksum_in_yaml() {
    local dataset_name="$1"
    local new_checksum="$2"
    
    if command -v yq &> /dev/null; then
        # Use yq to update in place
        yq eval ".datasets.$dataset_name.sha256 = \"$new_checksum\"" -i "$CONFIG_FILE"
    else
        # Use sed as fallback (less reliable)
        echo -e "${YELLOW}Warning: Using sed fallback for YAML update${NC}"
        echo -e "${YELLOW}Consider installing yq for better YAML handling${NC}"
        
        # Create a temporary file with updated checksum
        local temp_file=$(mktemp)
        python3 - << EOF > "$temp_file"
import yaml
import sys

with open('$CONFIG_FILE', 'r') as f:
    data = yaml.safe_load(f)

if 'datasets' in data and '$dataset_name' in data['datasets']:
    data['datasets']['$dataset_name']['sha256'] = '$new_checksum'

with open('$CONFIG_FILE', 'w') as f:
    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
EOF
        mv "$temp_file" "$CONFIG_FILE"
    fi
}

# Get dataset info from YAML
get_dataset_info() {
    local dataset_name="$1"
    local field="$2"
    
    if command -v yq &> /dev/null; then
        yq eval ".datasets.$dataset_name.$field" "$CONFIG_FILE"
    else
        python3 - << EOF
import yaml
with open('$CONFIG_FILE', 'r') as f:
    data = yaml.safe_load(f)
result = data.get('datasets', {}).get('$dataset_name', {}).get('$field')
if result is not None:
    print(result)
EOF
    fi
}

# Process all datasets
echo -e "\n${BLUE}Processing datasets...${NC}"

updated_count=0
skipped_count=0

# Get all dataset names
if command -v yq &> /dev/null; then
    dataset_names=$(yq eval '.datasets | keys | .[]' "$CONFIG_FILE")
else
    dataset_names=$(python3 - << EOF
import yaml
with open('$CONFIG_FILE', 'r') as f:
    data = yaml.safe_load(f)
for name in data.get('datasets', {}):
    print(name)
EOF
)
fi

for dataset in $dataset_names; do
    echo -e "\n${YELLOW}Processing:${NC} $dataset"
    
    # Get file path
    output_path=$(get_dataset_info "$dataset" "output_path")
    if [ -z "$output_path" ]; then
        echo -e "${RED}  ✗ No output_path found in config${NC}"
        skipped_count=$((skipped_count + 1))
        continue
    fi
    
    full_path="$TEST_DATA_DIR/$output_path"
    
    # Check if file exists
    if [ ! -f "$full_path" ]; then
        echo -e "${YELLOW}  ⚠ File not found: $output_path${NC}"
        skipped_count=$((skipped_count + 1))
        continue
    fi
    
    # Calculate current checksum
    echo "  Calculating SHA256..."
    current_checksum=$(calculate_sha256 "$full_path")
    
    # Get expected checksum from config
    expected_checksum=$(get_dataset_info "$dataset" "sha256")
    
    echo "  Current:  $current_checksum"
    echo "  Expected: $expected_checksum"
    
    # Compare checksums
    if [ "$current_checksum" = "$expected_checksum" ]; then
        echo -e "${GREEN}  ✓ Checksum matches${NC}"
        skipped_count=$((skipped_count + 1))
    else
        echo -e "${YELLOW}  ↻ Updating checksum...${NC}"
        update_checksum_in_yaml "$dataset" "$current_checksum"
        echo -e "${GREEN}  ✓ Updated${NC}"
        updated_count=$((updated_count + 1))
    fi
done

# Summary
echo -e "\n${GREEN}Summary${NC}"
echo "======="
echo "Updated: $updated_count"
echo "Skipped: $skipped_count"

if [ $updated_count -gt 0 ]; then
    echo -e "\n${GREEN}✓ Checksums updated in $CONFIG_FILE${NC}"
    echo -e "${YELLOW}Remember to commit the updated configuration file${NC}"
else
    echo -e "\n${GREEN}✓ All checksums were already up to date${NC}"
fi