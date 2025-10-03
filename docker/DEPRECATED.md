# ⚠️ DEPRECATED

**This directory is deprecated and should not be used for new development.**

## Why is this deprecated?

The container images in this directory are being deprecated because:

1. **CI/CD uses devcontainer**: The GitHub Actions workflows use `.devcontainer/Dockerfile` for testing and building
2. **Different source dependencies**: These images will soon depend on different source code
3. **Inconsistent with development workflow**: The devcontainer approach provides better integration with modern development tools

## What should you use instead?

For development and deployment, use:
- **Development**: `.devcontainer/Dockerfile` for VSCode devcontainer development
- **Deployment**: Container images published to GitHub Container Registry (GHCR) built from the devcontainer
- **CI/CD**: Automated workflows in `.github/workflows/`

## Timeline

This directory will be removed or archived in a future release.

## Questions?

See the main [README.md](../README.md) for current deployment instructions.
