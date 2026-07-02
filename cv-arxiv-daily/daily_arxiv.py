import datetime
import json
import os
import re

import arxiv

try:
    import yaml  # PyYAML — optional; falls back to the built-in defaults below.
except ImportError:  # pragma: no cover
    yaml = None

# ---------------------------------------------------------------------------
# arXiv client.
# Build ONE client and reuse it for the whole job so its built-in throttling and
# retries apply across every query. `Search.results()` was deprecated in v1.2 and
# REMOVED in arxiv v4.0.0 — the supported API is `client.results(search)`.
# ---------------------------------------------------------------------------
CLIENT = arxiv.Client(
    page_size=100,      # results per API page (max 2000)
    delay_seconds=3.0,  # arXiv asks for >= 3s between page fetches
    num_retries=5,      # retry flaky / empty pages (known API instability)
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Papers With Code was sunset by Meta on 2025-07-24 and its API is dead, so there
# is no live "official code" service any more. Instead we best-effort pull a repo
# link that the authors advertised in the arXiv comment / abstract — no API key,
# no external host, no rate limit.
GITHUB_RE = re.compile(r"https?://github\.com/[\w.\-]+/[\w.\-]+", re.IGNORECASE)

# Top-tier venues authors commonly note in the arXiv "comment" field, e.g.
# "Accepted to ICRA 2025", "CVPR2024", "To appear in T-RO".
VENUE_RE = re.compile(
    r"\b("
    r"CVPR|ICCV|ECCV|NeurIPS|NIPS|ICML|ICLR|AAAI|"   # ML / vision conferences
    r"ICRA|IROS|RSS|CoRL|"                            # robotics conferences
    r"T-?RO|TRO|RA-?L|RAL|IJRR|T-?PAMI|TPAMI|"        # journals
    r"SIGGRAPH|3DV|WACV|BMVC"
    r")\b[\s'’]*(\d{2,4})?",
    re.IGNORECASE,
)

# Fallback keyword set, used when config.yaml is missing or PyYAML is unavailable.
# Keys are section titles; values are arXiv query strings.
DEFAULT_KEYWORDS = {
    # General & sensor modality
    "SLAM": "SLAM",
    "Visual SLAM / VO": (
        '"visual SLAM" OR "visual odometry" OR "monocular SLAM" OR '
        '"stereo SLAM" OR "RGB-D SLAM"'
    ),
    "LiDAR SLAM": (
        '"LiDAR SLAM" OR "LiDAR odometry" OR "LiDAR-inertial" OR '
        '"laser SLAM" OR "lidar-based SLAM"'
    ),
    "Visual-Inertial SLAM": (
        '"visual-inertial odometry" OR "visual-inertial SLAM" OR "VIO" OR '
        '"visual inertial navigation"'
    ),
    # Problem type
    "Semantic SLAM": (
        '"semantic SLAM" OR "object SLAM" OR "object-level SLAM" OR '
        '("semantic mapping" AND SLAM)'
    ),
    "Dynamic SLAM": (
        '"dynamic SLAM" OR ("dynamic environment" AND (SLAM OR "visual odometry")) '
        'OR ("moving objects" AND SLAM)'
    ),
    "Active SLAM": (
        '"active SLAM" OR ("active mapping" AND (SLAM OR robot OR exploration)) OR '
        '("active perception" AND (SLAM OR robot)) OR '
        '("autonomous exploration" AND (SLAM OR mapping OR robot))'
    ),
    "Continual / Lifelong SLAM": (
        '"lifelong SLAM" OR "continual SLAM" OR "long-term SLAM" OR '
        '"lifelong localization" OR "lifelong mapping"'
    ),
    "Collaborative / Multi-Robot SLAM": (
        '"collaborative SLAM" OR "multi-robot SLAM" OR "multi-agent SLAM" OR '
        '"distributed SLAM" OR "swarm SLAM"'
    ),
    # Modern representations
    "Gaussian Splatting SLAM": (
        '"Gaussian Splatting SLAM" OR "GS-SLAM" OR "Gaussian SLAM" OR '
        '(("Gaussian Splatting" OR "3D Gaussian") AND (SLAM OR "visual odometry"))'
    ),
    "Foundation-SLAM (VLA/VLM)": (
        '("vision-language-action" OR "vision language action" OR "VLA model" OR '
        '"vision-and-language navigation" OR "vision-language navigation" OR '
        '"foundation model" OR "large language model" OR "vision-language model") AND '
        '(SLAM OR "visual odometry" OR "robot navigation" OR "autonomous navigation" '
        'OR "embodied navigation" OR "embodied agent" OR manipulation OR robotic)'
    ),
    "NeRF / Implicit SLAM": (
        '("NeRF" OR "neural radiance field" OR "neural implicit" OR '
        '"implicit neural representation") AND '
        '(SLAM OR "visual odometry" OR mapping OR reconstruction)'
    ),
    # Related computer-vision topics
    "SFM": 'SFM OR "Structure from Motion"',
    "Visual Localization": (
        '"Camera Localization" OR "Visual Localization" OR '
        '"Camera Re-localisation" OR "Loop Closure Detection" OR '
        '"visual place recognition" OR "image retrieval"'
    ),
    "Keypoint Detection": '"Keypoint Detection" OR "Feature Descriptor"',
    "Image Matching": '"Image Matching" OR "Keypoint Matching"',
}
DEFAULT_MAX_RESULTS = 10


def load_config():
    """Read keywords / max_results from config.yaml, falling back to defaults."""
    cfg_path = os.path.join(BASE_DIR, "config.yaml")
    if yaml is not None and os.path.exists(cfg_path):
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        keywords = cfg.get("keywords") or DEFAULT_KEYWORDS
        max_results = int(cfg.get("max_results", DEFAULT_MAX_RESULTS))
        return keywords, max_results
    return DEFAULT_KEYWORDS, DEFAULT_MAX_RESULTS


def get_authors(authors, first_author=False):
    if first_author is False:
        return ", ".join(str(author) for author in authors)
    return authors[0]


def sort_papers(papers):
    output = dict()
    for key in sorted(papers.keys(), reverse=True):
        output[key] = papers[key]
    return output


def to_anchor(text):
    """GitHub-style heading slug so the table of contents actually jumps."""
    anchor = text.strip().lower().replace(" ", "-")
    return re.sub(r"[^a-z0-9\-]", "", anchor)


def find_code_link(result):
    """Best-effort GitHub repo link from the arXiv comment / abstract."""
    for text in (getattr(result, "comment", None) or "",
                 getattr(result, "summary", "") or ""):
        match = GITHUB_RE.search(text)
        if match:
            return match.group(0).rstrip(").,;")
    return None


# Canonical display casing for detected venues (default is upper-case).
VENUE_CANONICAL = {
    "NIPS": "NeurIPS", "NEURIPS": "NeurIPS",
    "TRO": "T-RO", "T-RO": "T-RO",
    "RAL": "RA-L", "RA-L": "RA-L",
    "TPAMI": "T-PAMI", "T-PAMI": "T-PAMI",
    "SIGGRAPH": "SIGGRAPH",
}


def find_venue(result):
    """Detect an accepting conference / journal noted in the arXiv comment."""
    comment = getattr(result, "comment", None) or ""
    match = VENUE_RE.search(comment)
    if not match:
        return None
    venue = match.group(1).upper()
    venue = VENUE_CANONICAL.get(venue, venue)
    year = match.group(2)
    return f"{venue} {year}".strip() if year else venue


def get_daily_papers(topic, query="slam", max_results=10):
    """
    @param topic: str  section title
    @param query: str  arXiv query string
    @return (data, data_web): dicts keyed by topic
    """
    content = dict()
    content_to_web = dict()

    search = arxiv.Search(
        query=query,
        max_results=max_results,
        sort_by=arxiv.SortCriterion.SubmittedDate,
    )

    for result in CLIENT.results(search):
        paper_id = result.get_short_id()
        paper_title = result.title
        paper_url = result.entry_id
        paper_first_author = get_authors(result.authors, first_author=True)
        update_time = result.updated.date()

        print("Time =", update_time, " title =", paper_title,
              " author =", paper_first_author)

        # eg: 2108.09112v1 -> 2108.09112
        ver_pos = paper_id.find("v")
        paper_key = paper_id if ver_pos == -1 else paper_id[0:ver_pos]

        repo_url = find_code_link(result)
        venue = find_venue(result)

        code_cell = f"**[link]({repo_url})**" if repo_url else "null"
        title_cell = f"**{paper_title}**"
        if venue:
            title_cell += f" `{venue}`"

        content[paper_key] = (
            f"|**{update_time}**|{title_cell}|{paper_first_author} et.al."
            f"|[{paper_id}]({paper_url})|{code_cell}|\n"
        )

        web = (f"- {update_time}, **{paper_title}**, {paper_first_author} et.al., "
               f"Paper: [{paper_url}]({paper_url})")
        if repo_url:
            web += f", Code: **[{repo_url}]({repo_url})**"
        if venue:
            web += f", Venue: **{venue}**"
        content_to_web[paper_key] = web + "\n"

    return {topic: content}, {topic: content_to_web}


def update_json_file(filename, data_all):
    with open(filename, "r") as f:
        content = f.read()
        m = {} if not content else json.loads(content)

    json_data = m.copy()

    for data in data_all:
        for keyword in data.keys():
            papers = data[keyword]
            if keyword in json_data.keys():
                json_data[keyword].update(papers)
            else:
                json_data[keyword] = papers

    with open(filename, "w") as f:
        json.dump(json_data, f)


def json_to_md(filename, md_filename,
               to_web=False,
               use_title=True,
               use_tc=True,
               show_badge=True):
    """Render the accumulated JSON into a Markdown file."""

    DateNow = str(datetime.date.today()).replace("-", ".")

    with open(filename, "r") as f:
        content = f.read()
        data = {} if not content else json.loads(content)

    # clean the target file if it already exists, else create it
    with open(md_filename, "w+"):
        pass

    with open(md_filename, "a+") as f:

        if use_title and to_web:
            f.write("---\n" + "layout: default\n" + "---\n\n")

        if show_badge:
            f.write("[![Contributors][contributors-shield]][contributors-url]\n")
            f.write("[![Forks][forks-shield]][forks-url]\n")
            f.write("[![Stargazers][stars-shield]][stars-url]\n")
            f.write("[![Issues][issues-shield]][issues-url]\n\n")

        if use_title:
            f.write("## Updated on " + DateNow + "\n\n")
        else:
            f.write("> Updated on " + DateNow + "\n\n")

        # table of contents
        if use_tc:
            f.write("<details>\n")
            f.write("  <summary>Table of Contents</summary>\n")
            f.write("  <ol>\n")
            for keyword in data.keys():
                if not data[keyword]:
                    continue
                f.write(f"    <li><a href=#{to_anchor(keyword)}>{keyword}</a></li>\n")
            f.write("  </ol>\n")
            f.write("</details>\n\n")

        for keyword in data.keys():
            day_content = data[keyword]
            if not day_content:
                continue
            # section heading
            f.write(f"## {keyword}\n\n")

            if use_title:
                if not to_web:
                    f.write("|Publish Date|Title|Authors|PDF|Code|\n"
                            + "|---|---|---|---|---|\n")
                else:
                    f.write("| Publish Date | Title | Authors | PDF | Code |\n")
                    f.write("|:---------|:-----------------------|:---------|:------|:------|\n")

            # sort papers by date (newest first)
            day_content = sort_papers(day_content)
            for _, v in day_content.items():
                if v is not None:
                    f.write(v)

            f.write("\n")

            # back-to-top link
            f.write(f"<p align=right>(<a href=#{to_anchor('Updated on ' + DateNow)}>back to top</a>)</p>\n\n")

        if show_badge:
            repo = "thanhnguyencanh/SLAM-Resources"
            f.write(f"[contributors-shield]: https://img.shields.io/github/contributors/{repo}.svg?style=for-the-badge\n")
            f.write(f"[contributors-url]: https://github.com/{repo}/graphs/contributors\n")
            f.write(f"[forks-shield]: https://img.shields.io/github/forks/{repo}.svg?style=for-the-badge\n")
            f.write(f"[forks-url]: https://github.com/{repo}/network/members\n")
            f.write(f"[stars-shield]: https://img.shields.io/github/stars/{repo}.svg?style=for-the-badge\n")
            f.write(f"[stars-url]: https://github.com/{repo}/stargazers\n")
            f.write(f"[issues-shield]: https://img.shields.io/github/issues/{repo}.svg?style=for-the-badge\n")
            f.write(f"[issues-url]: https://github.com/{repo}/issues\n\n")

    print("finished")


if __name__ == "__main__":

    keywords, max_results = load_config()

    data_collector = []
    data_collector_web = []

    for topic, keyword in keywords.items():
        print("Keyword: " + topic)
        data, data_web = get_daily_papers(topic, query=keyword, max_results=max_results)
        data_collector.append(data)
        data_collector_web.append(data_web)
        print("\n")

    # 1. update README.md
    update_json_file(os.path.join(BASE_DIR, "cv-arxiv-daily.json"), data_collector)
    json_to_md(os.path.join(BASE_DIR, "cv-arxiv-daily.json"),
               os.path.join(BASE_DIR, "README.md"))

    # 2. update docs/index.md (GitHub Pages)
    update_json_file(os.path.join(BASE_DIR, "docs", "cv-arxiv-daily-web.json"), data_collector)
    json_to_md(os.path.join(BASE_DIR, "docs", "cv-arxiv-daily-web.json"),
               os.path.join(BASE_DIR, "docs", "index.md"), to_web=True)

    # 3. update docs/wechat.md
    update_json_file(os.path.join(BASE_DIR, "docs", "cv-arxiv-daily-wechat.json"), data_collector_web)
    json_to_md(os.path.join(BASE_DIR, "docs", "cv-arxiv-daily-wechat.json"),
               os.path.join(BASE_DIR, "docs", "wechat.md"), to_web=False, use_title=False)
