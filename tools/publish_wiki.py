#!/usr/bin/env python3
#
# Copyright 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Publish docs/tutorials/ as the GitHub wiki.

usage: tools/publish_wiki.py WIKI_CLONE_DIR [REPO_URL] [BRANCH]
  WIKI_CLONE_DIR  a clone of <repo>.wiki.git -- GitHub creates that repository
                  only once the wiki's first page has been saved in the web UI
  REPO_URL        default https://github.com/soumics/drcsim
  BRANCH          default ros2-jazzy-harmonic
Writes Home.md (the tutorials index), one page per tutorial and _Sidebar.md,
with links rewritten for the wiki: tutorial -> wiki page, other repo files ->
their GitHub page on BRANCH. A GitHub wiki belongs to the whole repository,
not a branch, so every page says which branch it documents. Then commit and
push in WIKI_CLONE_DIR yourself.
"""

import pathlib
import re
import sys

DOCS = pathlib.Path(__file__).resolve().parent.parent / 'docs' / 'tutorials'
LINK = re.compile(r'\]\(([^)#\s]+)(#[^)]*)?\)')


def page_name(path):
    """Return the wiki page name for a tutorial file (README is Home)."""
    return 'Home' if path.name == 'README.md' else path.stem


def rewrite(text, source, repo, branch):
    """Rewrite relative links in a tutorial's markdown for the wiki."""
    def fix(match):
        target, anchor = match.group(1), match.group(2) or ''
        if re.match(r'[a-z]+://', target):
            return match.group(0)
        resolved = (source.parent / target).resolve()
        if resolved.parent == DOCS and resolved.suffix == '.md':
            return f']({page_name(resolved)}{anchor})'
        root = DOCS.parent.parent
        relative = resolved.relative_to(root).as_posix()
        kind = 'tree' if resolved.is_dir() else 'blob'
        return f']({repo}/{kind}/{branch}/{relative}{anchor})'
    return LINK.sub(fix, text)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = pathlib.Path(sys.argv[1])
    repo = sys.argv[2] if len(sys.argv) > 2 else 'https://github.com/soumics/drcsim'
    branch = sys.argv[3] if len(sys.argv) > 3 else 'ros2-jazzy-harmonic'
    banner = (f'> These tutorials are for the [`{branch}`]({repo}/tree/{branch}) branch '
              '(ROS 2 Jazzy + Gazebo Harmonic), not the repository\'s default branch.\n\n')
    sidebar = [f'**[Tutorials](Home)** -- [`{branch}`]({repo}/tree/{branch})', '']
    for source in sorted(DOCS.glob('*.md')):
        text = rewrite(source.read_text(), source, repo, branch)
        (out / f'{page_name(source)}.md').write_text(banner + text)
        print(f'{source.name} -> {page_name(source)}.md')
        if source.name != 'README.md':
            title = source.read_text().splitlines()[0].lstrip('# ').strip()
            sidebar.append(f'- [{title}]({page_name(source)})')
    (out / '_Sidebar.md').write_text('\n'.join(sidebar) + '\n')


if __name__ == '__main__':
    main()
