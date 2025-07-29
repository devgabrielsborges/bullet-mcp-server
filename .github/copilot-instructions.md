# PyBullet MCP Server - Copilot Instructions

<!-- Use this file to provide workspace-specific custom instructions to Copilot. For more details, visit https://code.visualstudio.com/docs/copilot/copilot-customization#_use-a-githubcopilotinstructionsmd-file -->

## Project Overview

This is a Model Context Protocol (MCP) server that provides physics simulation capabilities using PyBullet. The server allows external applications to interact with PyBullet physics simulations through standardized MCP protocol endpoints.

## Key Technologies

- **PyBullet**: Physics simulation engine
- **MCP (Model Context Protocol)**: Communication protocol for AI model interactions
- **Python**: Primary programming language
- **uv**: Package and project management

## Architecture

- The server runs in headless mode (DIRECT) by default for compatibility with MCP clients
- Maintains simulation state including objects, physics parameters, and simulation status
- Provides tools for creating objects, applying forces, stepping simulation, and querying state
- Uses JSON serialization for complex data structures in responses

## Development Guidelines

1. Always ensure the physics client is initialized before performing operations
2. Handle PyBullet exceptions gracefully and return meaningful error messages
3. Use proper JSON serialization for complex objects and simulation state
4. Maintain object registry for tracking simulation entities
5. Follow MCP protocol specifications for tool definitions and responses

## Available Tools

- `create_simulation`: Initialize physics world
- `load_object`: Load URDF objects into simulation
- `set_object_color`: Change object appearance
- `apply_force`: Apply forces to objects
- `set_velocity`: Set object velocities
- `step_simulation`: Advance physics simulation
- `get_object_info`: Query object state
- `create_random_scene`: Generate test scenarios
- `reset_object_pose`: Reset object positions

## Resources

- Simulation status and parameters
- Object listings and individual object details
- Physics world configuration

You can find more info and examples at https://modelcontextprotocol.io/llms-full.txt

For PyBullet reference: https://pybullet.org/wordpress/
