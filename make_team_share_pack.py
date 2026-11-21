#!/usr/bin/env python3
"""
make_team_share_pack.py - Prepares complete team handover files and zip
"""

import os
import zipfile
import markdown

def generate_html_explainer():
    md_file = "TEAM_EXPLAINER_AND_HANDOVER.md"
    html_file = "TEAM_EXPLAINER_AND_HANDOVER.html"
    
    if not os.path.exists(md_file):
        print(f"Error: {md_file} not found.")
        return

    with open(md_file, "r", encoding="utf-8") as f:
        md_content = f.read()

    # Convert markdown to HTML with tables and code fences
    body = markdown.markdown(md_content, extensions=['tables', 'fenced_code', 'toc'])

    html = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>XR Hackathon 2026 - Track 3: Path Finder | Team Explainer & Handover</title>
    <style>
        :root {
            --primary: #2563eb;
            --primary-dark: #1d4ed8;
            --bg: #f8fafc;
            --card-bg: #ffffff;
            --text: #1e293b;
            --text-muted: #64748b;
            --border: #e2e8f0;
            --code-bg: #0f172a;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            line-height: 1.65;
            color: var(--text);
            background-color: var(--bg);
            margin: 0;
            padding: 40px 20px;
        }
        .container {
            max-width: 950px;
            margin: 0 auto;
            background: var(--card-bg);
            padding: 45px;
            border-radius: 12px;
            box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1), 0 2px 4px -2px rgba(0,0,0,0.1);
            border: 1px solid var(--border);
        }
        h1, h2, h3, h4 {
            color: #0f172a;
            font-weight: 700;
        }
        h1 {
            font-size: 2.2rem;
            border-bottom: 2px solid var(--primary);
            padding-bottom: 12px;
            margin-top: 0;
        }
        h2 {
            font-size: 1.5rem;
            margin-top: 2rem;
            border-bottom: 1px solid var(--border);
            padding-bottom: 8px;
        }
        h3 {
            font-size: 1.25rem;
            margin-top: 1.5rem;
        }
        p, li {
            font-size: 1.05rem;
        }
        table {
            border-collapse: collapse;
            width: 100%;
            margin: 24px 0;
            font-size: 0.95rem;
        }
        th, td {
            border: 1px solid var(--border);
            padding: 12px 14px;
            text-align: left;
        }
        th {
            background-color: #eff6ff;
            color: var(--primary-dark);
            font-weight: 600;
        }
        tr:nth-child(even) {
            background-color: #f8fafc;
        }
        code {
            background-color: #f1f5f9;
            color: #0f172a;
            padding: 3px 6px;
            border-radius: 4px;
            font-size: 0.9em;
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
        }
        pre {
            background-color: var(--code-bg);
            color: #f8fafc;
            padding: 18px;
            border-radius: 8px;
            overflow-x: auto;
            font-size: 0.92rem;
        }
        pre code {
            background-color: transparent;
            color: inherit;
            padding: 0;
        }
        blockquote {
            border-left: 4px solid var(--primary);
            padding: 12px 20px;
            margin: 20px 0;
            background-color: #eff6ff;
            border-radius: 0 8px 8px 0;
            color: #1e3a8a;
        }
        hr {
            border: 0;
            height: 1px;
            background: var(--border);
            margin: 30px 0;
        }
        a {
            color: var(--primary);
            text-decoration: none;
            font-weight: 500;
        }
        a:hover {
            text-decoration: underline;
        }
    </style>
</head>
<body>
    <div class="container">
""" + body + """
    </div>
</body>
</html>
"""
    with open(html_file, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[1] Created interactive HTML explainer: {html_file}")

def create_full_team_zip():
    zip_name = "PRISMHACK_Complete_Team_Package.zip"
    
    # Files to include in the team package
    include_files = [
        "TEAM_EXPLAINER_AND_HANDOVER.md",
        "TEAM_EXPLAINER_AND_HANDOVER.html",
        "README.md",
        "EXPLANATORY_DOCUMENT.md",
        "IMPLEMENTATION_PLAN.md",
        "main.py",
        "degrade_imu_default.py",
        "degrade_imu.py",
        "generate_dataset.py",
        "evaluate.py",
        "run_benchmarks.py",
        "test_suite.py",
        "package_submission.py",
        "metadata.json",
        "requirements.txt",
        "clean_imu.csv",
        "degraded_imu.csv",
        "checkpoints.csv",
        "ground_truth_trajectory.txt",
        "estimated_trajectory.txt",
        "CollegeName_TeamName.zip"
    ]

    # Directories to include
    include_dirs = [
        "src",
        "presentation",
        "demo",
        "results",
        "data"
    ]

    with zipfile.ZipFile(zip_name, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for f in include_files:
            if os.path.exists(f):
                zf.write(f, arcname=f)
                
        for d in include_dirs:
            if os.path.exists(d):
                for root, _, filenames in os.walk(d):
                    for fn in filenames:
                        fp = os.path.join(root, fn)
                        zf.write(fp, arcname=os.path.relpath(fp, "."))

    print(f"[2] Created complete team handover zip: {zip_name} ({os.path.getsize(zip_name):,} bytes)")

if __name__ == "__main__":
    generate_html_explainer()
    create_full_team_zip()
