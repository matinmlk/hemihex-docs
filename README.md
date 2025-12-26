# Docusaurus Prerequisites for Windows (Markdown + GitHub)

This document lists all required tools and setup steps to install and
run **Docusaurus on a Windows device** using **Markdown (MD) files** and
deploy to **GitHub Pages or a custom domain**.

------------------------------------------------------------------------

## 1. Install Node.js (Required)

Docusaurus is built on Node.js.

### Download (LTS Version Only)

https://nodejs.org

✅ Install **Node.js LTS (18.x or 20.x)**

### Verify Installation

``` bash
node -v
npm -v
```

------------------------------------------------------------------------

## 2. Install Git (Required for GitHub)

Git is required to push your documentation to GitHub.

### Download

https://git-scm.com

### Verify

``` bash
git --version
```

------------------------------------------------------------------------

## 3. Install a Code Editor (Recommended)

Best option: - ✅ **Visual Studio Code (VS Code)**
https://code.visualstudio.com

### Recommended Extensions

-   Markdown Preview
-   GitHub Markdown
-   ESLint (optional)

------------------------------------------------------------------------

## 4. Enable PowerShell Script Execution (If Blocked)

Run **once as Administrator**:

``` powershell
Set-ExecutionPolicy RemoteSigned
```

------------------------------------------------------------------------

## 5. GitHub Account Requirement

You must have: - ✅ A GitHub account - ✅ A GitHub repository for your
documentation (public or private)

------------------------------------------------------------------------

## 6. Install Docusaurus (After Prerequisites)

Run these commands:

``` bash
npx create-docusaurus@latest my-docs classic
cd my-docs
npm start
```

------------------------------------------------------------------------

## 7. Required Markdown Folder Structure

    my-docs/
    ├── docs/
    │   ├── intro.md
    │   ├── module1.md
    │   └── module2.md
    ├── docusaurus.config.ts
    ├── sidebars.ts
    └── package.json

------------------------------------------------------------------------

## 8. Internet & Firewall Requirements

The system must allow access to: - npm registry - GitHub - Node package
CDN

------------------------------------------------------------------------

## 9. Not Required

-   ❌ Python
-   ❌ Docker
-   ❌ Linux
-   ❌ WSL
-   ❌ Any database
-   ❌ Any web server

------------------------------------------------------------------------

## 10. Minimal Required Summary

  Requirement         Required
  ------------------- ----------
  Node.js LTS         ✅
  Git                 ✅
  VS Code             ✅
  GitHub Account      ✅
  PowerShell Access   ✅
