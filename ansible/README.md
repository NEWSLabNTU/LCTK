# LCTK Ansible Playbooks

This directory contains Ansible playbooks and roles for setting up the LCTK development environment.

## Structure

```
ansible/
├── ansible.cfg                        # Ansible configuration
├── ansible-galaxy-requirements.yaml   # Required Ansible collections
├── playbooks/                          # Ansible playbooks
│   └── lctk.dev_env.yaml              # Main development environment setup
├── roles/                              # Ansible roles
│   ├── lctk.dev_env.system_base/      # Base system packages
│   ├── lctk.dev_env.build_tools/      # Build and development tools
│   ├── lctk.dev_env.ros2/             # ROS 2 Humble installation
│   ├── lctk.dev_env.rust/             # Rust toolchain
│   ├── lctk.dev_env.python/           # Python environment
│   ├── lctk.dev_env.opencv/           # OpenCV libraries
│   ├── lctk.dev_env.gstreamer/        # GStreamer and plugins
│   ├── lctk.dev_env.geometric_libs/   # SFCGAL and geometry libraries
│   ├── lctk.dev_env.network_libs/     # libpcap and network tools
│   ├── lctk.dev_env.cuda/             # CUDA toolkit (optional)
│   ├── lctk.dev_env.dev_tools/        # Development tools (optional)
│   ├── lctk.dev_env.rosdep/           # ROS dependency management
│   └── lctk.dev_env.colcon_rust/      # Colcon extensions for Rust
├── ansible_collections/                # Downloaded collections (gitignored)
└── README.md
```

## Usage

The playbooks are executed through the `setup-dev-env.sh` script in the project root:

```bash
# Interactive installation
../setup-dev-env.sh

# Non-interactive installation
../setup-dev-env.sh -y

# Skip optional components
../setup-dev-env.sh -y --no-cuda --no-dev-tools

# Verbose output for debugging
../setup-dev-env.sh -v
```

## Direct Playbook Execution

You can also run the playbook directly with Ansible:

```bash
# Install Ansible first
pipx install ansible==6.*

# Install required collections (from ansible/ directory)
cd ansible/
ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml

# Run the playbook
ansible-playbook playbooks/lctk.dev_env.yaml

# With specific options
ansible-playbook playbooks/lctk.dev_env.yaml \
  --extra-vars "prompt_install_cuda=n" \
  --extra-vars "prompt_install_dev_tools=y"
```

## Adding New Roles

To add a new role:

1. Create the role structure:
   ```bash
   mkdir -p roles/lctk.dev_env.new_role/{tasks,defaults,meta}
   ```

2. Create the main task file:
   ```yaml
   # roles/lctk.dev_env.new_role/tasks/main.yaml
   ---
   - name: Your task name
     ansible.builtin.apt:
       name: package-name
       state: present
   ```

3. Add the role to the main playbook (`playbooks/lctk.dev_env.yaml`):
   ```yaml
   roles:
     # ...
     - role: lctk.dev_env.new_role
       when: some_condition
   ```

## Testing

To test the playbooks without making changes:

```bash
# From ansible/ directory
cd ansible/

# Dry run (check mode)
ansible-playbook playbooks/lctk.dev_env.yaml --check

# Test with verbose output
ansible-playbook playbooks/lctk.dev_env.yaml -vvv --check
```

## Requirements

- Ubuntu 22.04 LTS
- Python 3.8+
- Internet connection for downloading packages

## Troubleshooting

If the playbook fails:

1. Check the error message for the specific task that failed
2. Run with `-vvv` for verbose output
3. Ensure you have sudo privileges
4. Check internet connectivity
5. Verify Ubuntu version with `lsb_release -a`

For more information, see the main project README.