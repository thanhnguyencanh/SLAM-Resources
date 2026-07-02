# SLAM-Resources

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Arxiv Daily][arxiv-shield]][arxiv-url]

A curated collection of **SLAM (Simultaneous Localization and Mapping)** resources — papers, datasets, tutorials, tooling, and a **daily auto-updated arXiv feed** — organized by topic. Aimed at both beginners taking their first steps and researchers tracking the state of the art.

> 🤖 The [arXiv paper feed](cv-arxiv-daily/README.md) refreshes automatically every day via GitHub Actions. See [How the auto-update works](#-how-the-auto-update-works).

---

## 🔥 Daily arXiv Papers (auto-updated)

A bot fetches the newest papers from arXiv every day and writes them into browsable tables. **[→ Open the latest papers](cv-arxiv-daily/README.md)**

| Topic | What it tracks |
|---|---|
| **SLAM** | Simultaneous localization and mapping, visual/LiDAR/inertial odometry |
| **Foundation-SLAM (VLA/VLM)** | Foundation models, Vision-Language-Action & Vision-Language models for SLAM/robotics |
| **SFM** | Structure-from-Motion |
| **Visual Localization** | Camera relocalization, loop closure, visual place recognition, image retrieval |
| **Keypoint Detection** | Feature detectors & descriptors |
| **Image Matching** | Image / keypoint matching |
| **NeRF** | Neural radiance fields & implicit representations |

Keywords are configured in [`cv-arxiv-daily/config.yaml`](cv-arxiv-daily/config.yaml) — add a line to track a new topic.

---

## 📚 Contents

| Section | Description |
|---|---|
| [For Beginners](For-Beginner/README.md) | Roadmap, math, lectures, books and starter repos for learning SLAM from scratch |
| [Visual SLAM / VO](VSLAM/README.md) | Vision-based SLAM & visual-odometry libraries, datasets, tools and projects |
| [NeRF / Implicit SLAM](NeRF-SLAM/README.md) | Implicit-representation & NeRF papers for SLAM and robotics |
| [Active SLAM](Active-SLAM/) | Active-SLAM [paper list](Active-SLAM/Paper%20List/README.md) and the MIT [PALS seminar](Active-SLAM/MIT%20papers/README.md) reading list |
| [Datasets](Dataset/README.md) | Curated SLAM datasets that provide pose and map ground truth |
| [Calibration](Calibration/README.md) | LiDAR–IMU extrinsic/temporal calibration methods and toolboxes |
| [Docker](Docker/README.md) | Dockerfile cheat-sheet for building SLAM dependencies (Ceres, GTSAM, OpenCV, Pangolin, Livox…) |
| [Conference Papers](Papers/) | Accepted-paper lists for ICRA & IROS ([2022](Papers/ICRA2022/README.md)–[2024](Papers/ICRA2024/README.md)) |
| [arXiv Daily](cv-arxiv-daily/README.md) | The auto-updated daily paper feed described above |

---

## 🤖 How the auto-update works

The feed is powered by [`cv-arxiv-daily/daily_arxiv.py`](cv-arxiv-daily/daily_arxiv.py), a fork of [Vincentqyw/cv-arxiv-daily](https://github.com/Vincentqyw/cv-arxiv-daily):

1. A scheduled **GitHub Actions** workflow ([`.github/workflows/cv-arxiv-daily.yml`](.github/workflows/cv-arxiv-daily.yml)) runs once a day (and on demand).
2. The script queries the [arXiv API](https://info.arxiv.org/help/api/index.html) for each topic in [`config.yaml`](cv-arxiv-daily/config.yaml), resolves official code links, and annotates the accepting venue (ICRA/IROS/CVPR/T-RO…) when the authors note it.
3. It regenerates the Markdown tables and commits the changes back to `main`.

**Run it locally:**

```bash
cd cv-arxiv-daily
pip install -r requirements.txt
python daily_arxiv.py
```

**Enable the automation on your fork:** push these files, then in GitHub go to **Settings → Actions → General → Workflow permissions** and select **Read and write permissions** so the bot can commit. The schedule then runs on its own; you can also trigger it manually from the **Actions** tab. The schedule and steps live in [`.github/workflows/cv-arxiv-daily.yml`](.github/workflows/cv-arxiv-daily.yml).

---

## 🙏 Acknowledgements

This collection curates and builds on excellent community lists, including
[cv-arxiv-daily](https://github.com/Vincentqyw/cv-arxiv-daily),
[awesome-visual-slam](https://github.com/tzutalin/awesome-visual-slam),
[awesome-Implicit-NeRF-SLAM](https://github.com/DoongLi/awesome-Implicit-NeRF-SLAM),
[Awesome-LiDAR-IMU-calibration](https://github.com/Deephome/Awesome-LiDAR-Camera-Calibration),
and the [SLAM KR](https://www.facebook.com/groups/slamkr/) community. Thanks to all the original authors.

Contributions are welcome — open an issue or a pull request to add a resource.

[contributors-shield]: https://img.shields.io/github/contributors/thanhnguyencanh/SLAM-Resources.svg?style=for-the-badge
[contributors-url]: https://github.com/thanhnguyencanh/SLAM-Resources/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/thanhnguyencanh/SLAM-Resources.svg?style=for-the-badge
[forks-url]: https://github.com/thanhnguyencanh/SLAM-Resources/network/members
[stars-shield]: https://img.shields.io/github/stars/thanhnguyencanh/SLAM-Resources.svg?style=for-the-badge
[stars-url]: https://github.com/thanhnguyencanh/SLAM-Resources/stargazers
[issues-shield]: https://img.shields.io/github/issues/thanhnguyencanh/SLAM-Resources.svg?style=for-the-badge
[issues-url]: https://github.com/thanhnguyencanh/SLAM-Resources/issues
[arxiv-shield]: https://img.shields.io/github/actions/workflow/status/thanhnguyencanh/SLAM-Resources/cv-arxiv-daily.yml?style=for-the-badge&label=arxiv%20daily
[arxiv-url]: https://github.com/thanhnguyencanh/SLAM-Resources/actions/workflows/cv-arxiv-daily.yml
