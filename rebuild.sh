#!/bin/bash

# Function to handle errors
handle_error() {
    echo -e "\nError: $1\n"
    exit 1
}

# Ensure clean.sh is executable
echo -e "\nEnsuring clean.sh is executable...\n"
chmod +x ./clean.sh
if [ $? -ne 0 ]; then
    handle_error "Failed to make clean.sh executable."
fi

# Run the clean.sh script to clean the workspace
echo -e "\nCleaning the workspace...\n"
./clean.sh
if [ $? -ne 0 ]; then
    handle_error "Failed to clean the workspace."
fi

# Build the workspace using colcon
echo -e "\nBuilding the workspace...\n"
colcon build --symlink-install
if [ $? -ne 0 ]; then
    handle_error "Failed to build the workspace."
fi

echo -e "\nWorkspace build completed successfully.\n"
