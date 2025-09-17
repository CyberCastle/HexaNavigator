#!/bin/bash

# Script to recursively remove all __pycache__ directories
# This helps clean up Python bytecode cache files from the project

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Cleaning Python cache directories...${NC}"

# Find and count __pycache__ directories
CACHE_DIRS=$(find . -type d -name "__pycache__" 2>/dev/null)
COUNT=$(echo "$CACHE_DIRS" | grep -c . 2>/dev/null || echo "0")

if [ "$COUNT" -eq 0 ]; then
    echo -e "${GREEN}No __pycache__ directories found. Project is already clean.${NC}"
    exit 0
fi

echo -e "${YELLOW}Found $COUNT __pycache__ directories:${NC}"
echo "$CACHE_DIRS"
echo

# Ask for confirmation unless --force flag is provided
if [ "$1" != "--force" ] && [ "$1" != "-f" ]; then
    read -p "Do you want to remove all these directories? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Operation cancelled.${NC}"
        exit 0
    fi
fi

# Remove all __pycache__ directories
echo -e "${YELLOW}Removing __pycache__ directories...${NC}"
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true

# Also remove .pyc and .pyo files that might be scattered around
echo -e "${YELLOW}Removing scattered .pyc and .pyo files...${NC}"
find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete 2>/dev/null || true

echo -e "${GREEN}✓ Python cache cleanup completed successfully!${NC}"

# Show final status
REMAINING=$(find . -type d -name "__pycache__" 2>/dev/null | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo -e "${GREEN}✓ All __pycache__ directories have been removed.${NC}"
else
    echo -e "${RED}⚠ Warning: $REMAINING __pycache__ directories remain (possibly due to permissions).${NC}"
fi