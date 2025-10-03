# ⚠️ DEPRECATED

**This directory is deprecated and should not be used for new development or deployment.**

## Why is this deprecated?

The container launch scripts in this directory are being deprecated because:

1. **Gatekept container images**: These scripts reference GHCR images behind other accounts that you cannot control:
   - `ghcr.io/chandimafernando/erobs-common-img:latest` (personal account)
   - `ghcr.io/nsls2/erobs-ur-driver:latest`
   - `ghcr.io/nsls2/ur-hande-draft:latest`
   - `ghcr.io/nsls2/erobs-bsui:latest`

2. **Old architecture**: These containers use the deprecated `docker/` directory approach that clones and builds source code inside the container at build time

3. **Inconsistent with CI/CD**: The GitHub Actions workflows now use `.devcontainer/Dockerfile` for building and publishing to GHCR

## What should you use instead?

For deployment, use the new unified container workflow:

### Development
- Use `.devcontainer/Dockerfile` with VSCode devcontainer for local development

### Deployment
1. **Pull from GHCR**: The GitHub Actions workflow automatically builds and publishes container images to your repository's GHCR when you push version tags
2. **Run containers**: Launch ROS2 nodes directly from the unified container image using `ros2 launch` commands
3. **See the main README**: Refer to [../../README.md](../../README.md) for current deployment instructions

### CI/CD
- Automated builds are configured in `.github/workflows/docker-build.yaml`
- Push a version tag (e.g., `v1.0.0`) to trigger automated builds

## Timeline

This directory will be removed or archived in a future release.

## Questions?

See the main [README.md](../../README.md) for current deployment instructions.
