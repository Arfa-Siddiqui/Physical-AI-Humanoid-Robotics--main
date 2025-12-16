# Physical AI & Humanoid Robotics

A comprehensive technical book covering ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action (VLA) systems for building autonomous humanoid robots.

This project was built using **Spec-Kit Plus** and **Claude Code** – an AI-assisted specification-driven development workflow.

## 📚 What's Inside

This book guides you through building intelligent physical systems:

- **Module 1**: ROS 2 Humble (nodes, topics, services, rclpy, URDF)
- **Module 2**: Digital Twin simulation (Gazebo, Unity, physics)
- **Module 3**: NVIDIA Isaac (Isaac Sim, Isaac ROS, Nav2)
- **Module 4**: Vision-Language-Action (Whisper, GPT-4, complete autonomous pipeline)

## 🚀 Quick Start

### Prerequisites

- **Node.js** 20.0 or higher
- **npm** (comes with Node.js)
- **Git** for version control

### Local Development

1. **Clone the repository**
   ```bash
   git clone https://github.com/GITHUB_USERNAME/REPO_NAME.git
   cd REPO_NAME
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Start the development server**
   ```bash
   npm start
   ```

   This opens `http://localhost:3000` with live reload.

4. **Build for production**
   ```bash
   npm run build
   ```

   Static files are generated in the `build/` directory.

## 🔧 Configuration

### Replace Placeholders

Before deploying, replace the following placeholders in `docusaurus.config.js`:

1. **GITHUB_USERNAME** – Your GitHub username or organization name
2. **REPO_NAME** – Your repository name

**Find and replace:**
```javascript
// Before
url: 'https://GITHUB_USERNAME.github.io',
baseUrl: '/REPO_NAME/',
organizationName: 'GITHUB_USERNAME',
projectName: 'REPO_NAME',

// After (example)
url: 'https://yourusername.github.io',
baseUrl: '/Physical-AI-Book/',
organizationName: 'yourusername',
projectName: 'Physical-AI-Book',
```

Also update footer links in `docusaurus.config.js` and intro.md.

## 📦 Deployment to GitHub Pages

### Option 1: Automated Deployment (Recommended)

The project includes a GitHub Actions workflow that automatically builds and deploys on push.

1. **Enable GitHub Pages in your repository**:
   - Go to **Settings** → **Pages**
   - Source: **GitHub Actions**

2. **Push to main or your feature branch**:
   ```bash
   git add .
   git commit -m "Initial book setup"
   git push origin main
   ```

3. **GitHub Actions will automatically**:
   - Build the Docusaurus site
   - Deploy to GitHub Pages
   - Your site will be live at `https://GITHUB_USERNAME.github.io/REPO_NAME/`

### Option 2: Manual Deployment

```bash
# Set your GitHub username
GIT_USER=<Your GitHub username> npm run deploy
```

This builds the site and pushes to the `gh-pages` branch.

## 📂 Project Structure

```
Physical-AI-Humanoid-Robotics-/
├── .github/
│   └── workflows/
│       └── deploy.yml          # GitHub Actions deployment
├── docs/                        # Book content (Markdown)
│   ├── intro/                   # Introduction chapters
│   ├── foundations/             # Sensor, actuator, digital twin basics
│   ├── module-1-ros2/           # ROS 2 module
│   ├── module-2-simulation/     # Simulation module
│   ├── module-3-isaac/          # NVIDIA Isaac module
│   ├── module-4-vla/            # VLA module
│   ├── capstone/                # Capstone integration
│   └── code-examples/           # Executable code samples
│       ├── ros2/                # ROS 2 Python examples
│       ├── gazebo/              # Gazebo launch files
│       ├── isaac/               # Isaac Sim templates
│       ├── nav2/                # Nav2 configurations
│       └── vla/                 # VLA integration scripts
├── src/                         # Docusaurus theme customization
├── static/                      # Static assets (images, diagrams)
├── docusaurus.config.js         # Docusaurus configuration
├── sidebars.js                  # Sidebar navigation structure
└── package.json                 # Node.js dependencies

specs/001-physical-ai-book/      # Spec-Kit Plus artifacts (in repo root)
├── spec.md                      # Feature specification
├── plan.md                      # Implementation plan
├── tasks.md                     # Task breakdown
└── checklists/                  # Quality validation checklists

history/                         # Development history (in repo root)
├── prompts/                     # Prompt History Records (PHRs)
│   └── 001-physical-ai-book/
└── adr/                         # Architecture Decision Records
```

## 🛠️ Development Workflow

This book was built using **Spec-Kit Plus**:

1. **Specification** (`/sp.specify`) – Define requirements
2. **Clarification** (`/sp.clarify`) – Resolve ambiguities
3. **Planning** (`/sp.plan`) – Architecture and design
4. **Task Breakdown** (`/sp.tasks`) – Implementation tasks
5. **Implementation** (`/sp.implement`) – Execute tasks

All workflow artifacts are in `specs/001-physical-ai-book/` and `history/`.

## 🧪 Validation

### Build Validation
```bash
npm run build
```

Should exit with code 0 (no errors).

### Content Checks
- All chapters: 500-1200 words
- All code examples include usage instructions
- Mermaid diagrams render correctly
- Internal links resolve

## 📖 Reading the Book

### Prerequisites for Readers
- Basic Python programming
- Foundational AI/ML knowledge
- Command line familiarity

### Recommended Platform
- **Ubuntu 22.04 LTS** with ROS 2 Humble
- **Windows 10/11** with WSL2 (notes provided)
- **Optional**: NVIDIA GPU for Isaac Sim

## 🤝 Contributing

This book is open-source! Contributions welcome:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## 📄 License

**MIT / CC-BY** – Free to use, modify, and distribute with attribution.

Code examples are free to use without restriction.

## 🙏 Acknowledgments

Built with:
- [Docusaurus](https://docusaurus.io/) – Documentation framework
- [Spec-Kit Plus](https://github.com/) – Specification-driven development
- [Claude Code](https://claude.com/claude-code) – AI-assisted development
- [ROS 2](https://docs.ros.org/) – Robot Operating System
- [NVIDIA Isaac](https://developer.nvidia.com/isaac) – Robotics simulation platform

---

**Questions?** Open an issue on [GitHub](https://github.com/GITHUB_USERNAME/REPO_NAME/issues)

**Built with** ❤️ **using Spec-Kit Plus and Claude Code**
