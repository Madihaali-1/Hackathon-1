# Quickstart: ROS 2 Humanoid System Module

## Prerequisites

- Node.js 18 or higher
- npm or yarn package manager
- Git for version control

## Setup Instructions

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

3. Start the development server:
   ```bash
   npm start
   # or
   yarn start
   ```

4. Open your browser to http://localhost:3000 to view the documentation

## Adding the ROS 2 Module

1. The ROS 2 Humanoid System module consists of three chapters located in `docs/modules/ros2-humanoid-system/`
   - `intro-to-ros2.md` - Introduction to ROS 2 for Physical AI
   - `ros2-communication.md` - ROS 2 Communication Model
   - `urdf-robot-structure.md` - Robot Structure with URDF

2. The module is integrated into the Docusaurus sidebar via the sidebar configuration

## Building for Production

To build the documentation site for production:

```bash
npm run build
# or
yarn build
```

The built site will be available in the `build/` directory and can be deployed to any static hosting service.

## Running Tests

To run any available tests:

```bash
npm test
# or
yarn test
```

## Local Development Tips

- Use hot reloading during development: `npm start`
- Edit markdown files directly to update content
- The sidebar will automatically update based on the file structure
- Use Docusaurus' built-in components for better integration