# CI/CD Pipeline Documentation

This document describes the comprehensive CI/CD pipeline setup for the beacon positioning system project.

## Overview

The CI pipeline includes multiple workflows that run automatically on push and pull requests:

- **Main CI Pipeline** (`ci.yml`) - Core testing and quality checks
- **Quality Metrics** (`quality-metrics.yml`) - Code analysis and metrics
- **Security Scanning** (`security.yml`) - Security audits and vulnerability checks

## Workflows

### 1. Main CI Pipeline (`.github/workflows/ci.yml`)

Runs on every push and pull request to `main` and `develop` branches.

**Jobs:**
- **Test Suite**: Runs on multiple Rust versions (stable, beta)
  - Code formatting check (`cargo fmt`)
  - Clippy linting (`cargo clippy`)
  - Build all packages
  - Unit tests
  - Integration tests

- **Code Coverage**: Generates coverage reports using `cargo-llvm-cov`
  - Uploads to Codecov
  - Configured with `.codecov.yml`

- **Security Audit**: Checks for known vulnerabilities
  - Uses `cargo-audit`

- **Benchmarks**: Performance testing (main branch only)
  - Runs all benchmark suites
  - Stores results as artifacts

- **Quality Analysis**: Additional code quality checks
  - Outdated dependencies check
  - Unused dependencies check
  - Detailed clippy reports

- **Comprehensive Test Suite**: Runs the full test suite
  - Uses existing `run_tests.sh` script
  - Includes environmental and failure mode tests

- **Release Readiness**: Final validation (main branch only)
  - Confirms all checks passed
  - Generates release summary

### 2. Quality Metrics (`.github/workflows/quality-metrics.yml`)

Runs on push to main, pull requests, and weekly schedule.

**Jobs:**
- **Complexity Analysis**: Code statistics using `tokei`
- **Dependency Analysis**: Dependency tree and security advisories
- **Documentation Check**: Documentation coverage and quality
- **Performance Tracking**: Benchmark results tracking (main branch only)
- **Quality Summary**: Consolidated quality report

### 3. Security Scanning (`.github/workflows/security.yml`)

Runs on push, pull requests, and daily schedule.

**Jobs:**
- **Security Audit**: Vulnerability scanning with detailed reporting
- **Dependency License Check**: License compliance verification
- **Supply Chain Security**: Advanced security checks using `cargo-deny`
- **Security Summary**: Consolidated security report

## Code Coverage

Code coverage is handled by:
- `cargo-llvm-cov` for generating coverage data
- Codecov for reporting and tracking
- Configuration in `.codecov.yml`:
  - Target: 80% project coverage, 70% patch coverage
  - Ignores test files and benchmarks
  - Fails CI if coverage drops significantly

## Local Development

### Running CI Checks Locally

Before pushing code, run the local CI script to catch issues early:

**Linux/macOS:**
```bash
./ci-local.sh
```

**Windows:**
```powershell
.\ci-local.ps1
```

These scripts run the same checks as the CI pipeline:
1. Code formatting check
2. Clippy linting
3. Build all packages
4. Unit tests
5. Integration tests
6. Security audit (if installed)

### Installing Required Tools

For full local CI functionality, install these tools:

```bash
# Core tools
cargo install cargo-audit
cargo install cargo-outdated
cargo install cargo-udeps
cargo install cargo-llvm-cov
cargo install cargo-license
cargo install cargo-deny
cargo install tokei

# For nightly features (cargo-udeps)
rustup install nightly
```

## Artifacts and Reports

The CI pipeline generates several artifacts:

### Main CI Pipeline
- `benchmark-results` - Performance benchmark data
- `quality-reports` - Clippy and quality analysis reports
- `release-summary` - Release readiness report

### Quality Metrics
- `code-statistics` - Lines of code and complexity metrics
- `dependency-analysis` - Dependency tree and security info
- `documentation-quality` - Documentation coverage reports
- `performance-report` - Performance tracking data
- `quality-summary` - Consolidated quality metrics

### Security Scanning
- `security-report` - Vulnerability scan results
- `license-report` - Dependency license information
- `supply-chain-report` - Supply chain security analysis
- `security-summary` - Consolidated security report

## Configuration Files

### `.codecov.yml`
Configures code coverage reporting:
- Coverage targets and thresholds
- Files to ignore
- Comment and annotation settings

### `deny.toml` (auto-generated)
Configures supply chain security checks:
- Allowed/denied licenses
- Security advisory settings
- Dependency banning rules

## Monitoring and Maintenance

### Regular Tasks
1. **Weekly**: Review quality metrics reports
2. **Daily**: Check security scan results
3. **Monthly**: Update dependencies and tools
4. **Quarterly**: Review and update CI configuration

### Troubleshooting

**Common Issues:**

1. **Coverage drops**: Check if new code lacks tests
2. **Security alerts**: Update vulnerable dependencies
3. **Build failures**: Check for breaking changes in dependencies
4. **Benchmark failures**: Verify performance hasn't regressed

**Getting Help:**
- Check workflow logs in GitHub Actions tab
- Review artifact reports for detailed information
- Run local CI scripts to reproduce issues

## Best Practices

1. **Before Pushing:**
   - Run `./ci-local.sh` or `.\ci-local.ps1`
   - Ensure all tests pass locally
   - Check code formatting with `cargo fmt`

2. **Writing Tests:**
   - Maintain >80% code coverage
   - Include integration tests for critical paths
   - Add benchmarks for performance-critical code

3. **Dependencies:**
   - Regularly update dependencies
   - Review security advisories
   - Use specific version ranges

4. **Documentation:**
   - Document public APIs
   - Keep README files updated
   - Include examples in documentation

## Integration with Development Workflow

The CI pipeline integrates with:
- **Pull Requests**: All checks must pass before merging
- **Branch Protection**: Main branch requires CI success
- **Code Reviews**: Reviewers can see coverage and quality reports
- **Releases**: Release readiness check ensures quality

This comprehensive CI setup ensures code quality, security, and reliability while providing detailed feedback to developers.