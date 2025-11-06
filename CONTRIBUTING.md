# Contributing to Lododo Arm

Thank you for your interest in contributing to the Lododo Arm project! This document provides guidelines for contributing.

## 🌟 Ways to Contribute

- 🐛 Report bugs
- 💡 Suggest new features
- 📝 Improve documentation
- 🔧 Submit bug fixes
- ✨ Add new features
- 🧪 Write tests

## 📋 Before You Start

1. Check [existing issues](https://github.com/harryzy/lododo-arm/issues) to avoid duplicates
2. For major changes, open an issue first to discuss your proposal
3. Make sure you can build and run the project successfully

## 🔄 Development Workflow

### 1. Fork and Clone

```bash
# Fork on GitHub first, then:
mkdir -p ~/lododo-arm/src
cd ~/lododo-arm/src
git clone https://github.com/YOUR_USERNAME/lododo-arm.git .
cd ~/lododo-arm

# Add upstream remote

### 2. Create a Branch

```bash
# Update your main branch
git checkout main
git pull upstream main

# Create a feature branch
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

### 3. Make Changes

- Follow the existing code style
- Add comments for complex logic
- Update documentation as needed
- Test your changes thoroughly

### 4. Commit

```bash
# Stage your changes
git add .

# Commit with a descriptive message
git commit -m "Add feature: brief description"
```

**Commit Message Format:**
```
type: subject

body (optional)

footer (optional)
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Build/config changes

**Examples:**
```
feat: add cube edge length filtering in detection mode

Implemented smart filtering for cube candidates:
- Edge length range validation (3-8cm)
- Position range check (X <= 0.5m)
- Single candidate tolerance to avoid false negatives

Closes #123
```

### 5. Push and Create PR

```bash
# Push to your fork
git push origin feature/your-feature-name

# Go to GitHub and create a Pull Request
```

## 📝 Pull Request Guidelines

### PR Title
Use the same format as commit messages:
```
feat: add cube detection optimization
fix: resolve Y-axis inversion in URDF model
docs: update installation instructions
```

### PR Description
Include:
- **What**: What changes did you make?
- **Why**: Why are these changes needed?
- **How**: How did you implement them?
- **Testing**: How did you test the changes?
- **Screenshots**: If applicable (especially for UI changes)

### PR Checklist
- [ ] Code builds successfully
- [ ] All tests pass
- [ ] New tests added (if applicable)
- [ ] Documentation updated
- [ ] No new warnings or errors
- [ ] Follows existing code style
- [ ] Branch is up to date with main

## 🧪 Testing

### Run Tests
```bash
# Build with tests
colcon build --symlink-install

# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select arm_perception_yolo
```

### Manual Testing
- Test on real hardware (if available)
- Test in simulation (Gazebo)
- Check RViz visualization
- Verify all affected features work

## 💻 Code Style

### Python
- Follow PEP 8
- Use type hints where possible
- Maximum line length: 100 characters
- Use meaningful variable names

```python
# Good
def calculate_depth(disparity: float, baseline: float, focal_length: float) -> float:
    """Calculate object depth using triangulation."""
    return (focal_length * baseline) / disparity

# Avoid
def calc(d, b, f):
    return (f * b) / d
```

### C++
- Follow ROS2 style guide
- Use `snake_case` for functions and variables
- Use `PascalCase` for classes
- Always initialize variables

### ROS2 Conventions
- Topic names: lowercase with underscores
- Service names: lowercase with underscores
- Node names: lowercase with underscores
- Parameter names: lowercase with underscores

## 📦 Package Structure

When creating new packages:
```
my_package/
├── package.xml          # Package manifest
├── CMakeLists.txt       # Build configuration (C++)
├── setup.py             # Build configuration (Python)
├── config/              # Configuration files
├── launch/              # Launch files
├── src/                 # Source code (C++)
├── my_package/          # Python module
├── test/                # Unit tests
└── README.md            # Package documentation
```

## 🐛 Bug Reports

### Before Reporting
- Search existing issues
- Update to latest version
- Test in simulation first

### Bug Report Template
```markdown
## Description
Brief description of the bug

## Steps to Reproduce
1. Launch the system with...
2. Run the command...
3. Observe the error...

## Expected Behavior
What you expected to happen

## Actual Behavior
What actually happened

## Environment
- OS: Ubuntu 22.04
- ROS2: Humble
- Hardware: [Real robot / Simulation]
- Branch/Version: v0.972

## Logs
```
Paste relevant logs here
```

## Screenshots (if applicable)
```

## 💡 Feature Requests

### Feature Request Template
```markdown
## Problem Statement
What problem does this feature solve?

## Proposed Solution
How should this feature work?

## Alternatives Considered
What other solutions did you consider?

## Additional Context
Any other information, mockups, or examples
```

## 📚 Documentation

### Code Comments
- Use docstrings for all public functions/classes
- Explain **why**, not just **what**
- Include parameter types and return values

```python
def triangulate(det1: Detection2D, det2: Detection2D) -> Optional[MeasuredObject]:
    """
    Perform stereo triangulation to calculate 3D position and dimensions.
    
    Uses disparity between two camera views to calculate depth, then
    applies camera intrinsics to determine real-world position and size.
    
    Args:
        det1: Detection from first camera view
        det2: Detection from second camera view
        
    Returns:
        MeasuredObject with 3D position and dimensions, or None if
        triangulation fails (insufficient disparity, out of range, etc.)
    """
```

### Documentation Files
- Keep docs in the `docs/` folder
- Use Markdown format
- Include code examples
- Add images/diagrams when helpful

## ⚖️ License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.

## 🙏 Recognition

All contributors will be acknowledged in the project's README and release notes.

## 📞 Questions?

- Open an issue for questions
- Tag it with `question` label
- Provide context and what you've tried

## 🎉 Thank You!

Your contributions make this project better for everyone. We appreciate your time and effort!

---

**Happy Coding!** 🚀
