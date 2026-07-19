# CI/CD Pipelines

This repository contains several workflows that automate common tasks.


## 1. Publish Production And Development Images (`publish_image.yaml`)

Triggered automatically on pushes to `main` and `develop`, or manually via `workflow_dispatch`. This workflow builds and publishes production and development images.

* **Function:** Builds two types of images: **PROD** (ARM64) and **DEV** (AMD64).
* **Tagging Strategy:**
  * `latest-prod`: Generated only when pushing to `main`.
  * `<branch>-prod` / `<branch>-dev`: Branch-specific tags.
  * `sha-<commit_sha>-prod` / `sha-<commit_sha>-dev`: Immutable tags for tracking specific commits.


## 2. PR Pipeline (`pr_pipeline.yaml`)

Triggered on every Pull Request; used for automatic testing.

* **Function:** Performs a "quality gate" check. It detects if Dockerfiles have changed, builds a temporary image if necessary, and runs ROS 2 functional unit tests.
* **Tagging Strategy:**
  * `pr-<number>-dev`: Temporary tag for the PR (e.g., `pr-12-dev`).


---


## Maintenance & Utilities


### 3. GHCR Garbage Collector (`ghcr_garbage_collector.yaml`)

A scheduled daily task that automatically manages your registry storage.

* **Function:** Enforces retention policies to delete old and unnecessary images.
* **Schedule:** Runs daily at 00:00 UTC.
* **Policy:**
  * **DEV SHA tags:** Keeps the 5 most recent; deletes older than 14 days.
  * **PROD SHA tags:** Keeps the 10 most recent; deletes older than 30 days.
  * **Untagged Images:** Deletes anything untagged older than 3 days.


### 4. Cleanup Branch and PR Images (`cleanup_branch_images.yaml`)

Triggered when a PR is closed or a branch is deleted.

* **Function:** Performs automated "housekeeping" by removing specific tags created by that branch/PR (e.g., `pr-12-dev`).


### 5. Manual Image Cleanup Utility (`manual_image_cleanup.yaml`)

An on-demand tool triggered manually via the GitHub Actions UI.

* **Function:** Allows you to surgically clean up specific tags (even those with typos or incorrect naming) and optionally wipe untagged layers.
* **Usage:** Provide a list of tags or patterns (supports wildcards using `*`), choose whether to delete untagged images, and specify a retention period for the cleanup.


---


## Automation


### 6. Auto Assign PR Author (`auto_assign.yaml`)

Automatically adds the PR author as an assignee when a PR is opened.


### 7. Tag release on main (`tag_on_main.yaml`)

Automatically bumps the semantic version and creates a GitHub Release whenever code is merged to `main`.