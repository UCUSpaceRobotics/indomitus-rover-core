# Git Workflow and Branch Strategy

## Branch Structure

### Main Branches

- **`main`** - Stable, production-ready version of the rover
  - Only fully tested and approved code
  - Protected branch - no direct pushes allowed

- **`develop`** - Development integration branch
  - All feature branches are created from this branch
  - All completed features are merged back into this branch
  - Base branch for all team work

---

## Team Branch Naming Convention

### Branch Name Format

All team branches **must** follow this pattern:
```
{type}/{team}/{description}
```

### Branch Types

- `feature/` - New functionality
- `bugfix/` - Bug fixes
- `refactor/` - Code refactoring
- `experiment/` - Experimental features/tests
- `hotfix/` - Urgent fixes

### Team Names

- `arm` - Arm team
- `navigation` - Navigation team
- `science` - Science team
- `drone` - Drone team
- `suspension` - Suspension team
- `shared` - cross-team branch
