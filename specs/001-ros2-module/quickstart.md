# Quickstart: Setting up ROS 2 Documentation with Docusaurus

## Prerequisites

- Node.js version 18.0 or higher
- npm or yarn package manager
- Git for version control

## Installation Steps

### 1. Initialize Docusaurus Project

```bash
# Create a new Docusaurus project
npx create-docusaurus@latest website-name classic

# Or if you prefer yarn
yarn create docusaurus website-name classic

# Or if you prefer pnpm
pnpm create docusaurus@latest website-name classic
```

### 2. Navigate to Project Directory

```bash
cd website-name
```

### 3. Install Additional Dependencies (if needed)

```bash
npm install @docusaurus/module-type-aliases @docusaurus/types
```

### 4. Create Module 1 Directory Structure

```bash
mkdir -p docs/module-1
```

### 5. Create Chapter Files

Create the three required chapters in the `docs/module-1/` directory:

```bash
# Create Chapter 1: ROS 2 Basics
touch docs/module-1/chapter-1-ros2-basics.md

# Create Chapter 2: Python-ROS Bridge
touch docs/module-1/chapter-2-rclpy-bridge.md

# Create Chapter 3: Humanoid URDF
touch docs/module-1/chapter-3-urdf-modeling.md
```

### 6. Update Sidebar Configuration

Update `sidebars.js` to include the new module and chapters:

```javascript
// sidebars.js
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        {
          type: 'doc',
          id: 'module-1/chapter-1-ros2-basics',
          label: 'Chapter 1: ROS 2 Basics'
        },
        {
          type: 'doc',
          id: 'module-1/chapter-2-rclpy-bridge',
          label: 'Chapter 2: Python-ROS Bridge (rclpy)'
        },
        {
          type: 'doc',
          id: 'module-1/chapter-3-urdf-modeling',
          label: 'Chapter 3: Humanoid Description (URDF)'
        }
      ]
    }
  ]
};
```

### 7. Configure Docusaurus

Update `docusaurus.config.js` with appropriate site configuration:

```javascript
// docusaurus.config.js
module.exports = {
  title: 'ROS 2 Educational Module',
  tagline: 'The Robotic Nervous System',
  url: 'https://your-username.github.io',
  baseUrl: '/hackathon1/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'your-username', // Usually your GitHub org/user name
  projectName: 'hackathon1', // Usually your repo name
  // ... rest of configuration
};
```

### 8. Start Development Server

```bash
npm run start

# Or with yarn
yarn start

# Or with pnpm
pnpm start
```

### 9. Add Content to Chapters

Add content to each chapter file with appropriate frontmatter:

**docs/module-1/chapter-1-ros2-basics.md**:
```markdown
---
title: Chapter 1 - ROS 2 Basics
description: Introduction to ROS 2 as robotic middleware
sidebar_position: 1
---

# Chapter 1: ROS 2 Basics

## ROS 2 as Robotic Middleware

[Content about ROS 2 as middleware]

## Nodes, Topics, Services, Actions

[Content about these concepts]

## Robot Data Flow and Communication

[Content about data flow and communication]
```

### 10. Build for Production

```bash
npm run build

# Or with yarn
yarn build

# Or with pnpm
pnpm build
```

### 11. Deploy to GitHub Pages

```bash
npm run deploy

# Or with yarn
yarn deploy

# Or with pnpm
pnpm deploy
```

## Useful Commands

- `npm run start` - Start local development server
- `npm run build` - Build static files for production
- `npm run serve` - Serve the built site locally
- `npm run deploy` - Deploy to GitHub Pages
- `npm run clear` - Clear the Docusaurus cache

## Troubleshooting

### Common Issues

1. **Port already in use**: Use `npm run start -- --port 3001` to use a different port
2. **Build fails**: Check for syntax errors in Markdown files
3. **Images not loading**: Ensure images are in the `static/img/` directory and referenced correctly

### Development Tips

- Use hot reload during development (changes reflect immediately)
- Test on different screen sizes for responsive design
- Use proper heading hierarchy for accessibility
- Include alt text for all images