# Fix Isaac Sim Directory Permissions

This script (`fix_isaac_dirs_safe.sh`) ensures that all shared Isaac Sim directories have the correct ownership and permissions for multi-user access.

---

## ✅ What It Does

* Recursively scans all subdirectories under:

  * `/var/cache/isaac`
  * `/persistent/isaac`
* Identifies directories that:

  * Are not owned by `root:isaac`
  * Do not have the correct permissions (`2775`)
* Can either **report issues only** (`--dry-run`) or **apply fixes** (`--fix`).

---

## 🛠 Usage

### Dry Run (safe)

This will only report issues without making changes:

```bash
bash fix_isaac_dirs_safe.sh --dry-run
```

### Apply Fixes

You must run this as a user with `sudo` privileges:

```bash
sudo bash fix_isaac_dirs_safe.sh --fix
```

---

## 🔐 Required Permissions

* You must be in the `isaac` group to benefit from shared write access.
* Only users with `sudo` can apply ownership corrections.

---

## 🔄 When to Run This

Run this script if:

* A user cannot save logs, cache, or assets inside the container
* Isaac Sim errors mention permission issues or ownership mismatches
* You're setting up Isaac Sim on a new machine or adding new users

---

## 🧪 Example Output

```bash
❗ /var/cache/isaac/data/documents/Kit/shared: [fyy2ws:isaac 2755] → [root:isaac 2775]
✅ Done. Run with --fix to apply changes.
```

---

## 🔒 Notes

* All directories must be owned by `root:isaac`.
* The `setgid` bit (`2` in `2775`) ensures that new files created inside inherit the `isaac` group.
* This is important for shared environments like labs or servers.

---

## 📁 Affected Directories

```
/var/cache/isaac
/persistent/isaac
```

These include:

* Cache (`kit`, `ov`, `pip`, `glcache`, `computecache`)
* Logs
* Asset data
* Project documents

---

## 🙋 Need Help?

If you encounter unexplained permission errors in Isaac Sim, contact your lab admin or re-run this script with `--dry-run` to confirm status.
