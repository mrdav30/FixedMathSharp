import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT_PATH = pathlib.Path(__file__).with_name("rewrite-wiki-links-for-github-wiki.py")


def load_rewriter():
    spec = importlib.util.spec_from_file_location("rewrite_wiki_links", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RewriteWikiLinksForGithubWikiTests(unittest.TestCase):
    def test_rewrites_fixedmathsharp_wiki_page_links_without_md_extension(self):
        rewriter = load_rewriter()

        with tempfile.TemporaryDirectory() as temp_dir:
            root = pathlib.Path(temp_dir)
            wiki_dir = root / "wiki"
            repo_root = root / "repo"
            wiki_dir.mkdir()
            (repo_root / "docs" / "wiki").mkdir(parents=True)

            (wiki_dir / "Home.md").write_text("# Home\n", encoding="utf-8")
            (wiki_dir / "Overview.md").write_text(
                "\n".join(
                    [
                        "[Home](Home.md)",
                        "[Fixed64 Representation](fixed64-representation.md#raw-layout)",
                    ]
                ),
                encoding="utf-8",
            )
            (wiki_dir / "fixed64-representation.md").write_text(
                "# Fixed64 Representation\n",
                encoding="utf-8",
            )

            rewriter.rewrite_wiki_links(
                wiki_dir=wiki_dir,
                repository="mrdav30/FixedMathSharp",
                sync_sha="abc123",
                source_dir="docs/wiki",
                repo_root=repo_root,
            )

            content = (wiki_dir / "Overview.md").read_text(encoding="utf-8")

        self.assertIn("[Home](Home)", content)
        self.assertIn(
            "[Fixed64 Representation](fixed64-representation#raw-layout)",
            content,
        )

    def test_leaves_markdown_links_outside_wiki_source_unchanged(self):
        rewriter = load_rewriter()

        with tempfile.TemporaryDirectory() as temp_dir:
            root = pathlib.Path(temp_dir)
            wiki_dir = root / "wiki"
            repo_root = root / "repo"
            wiki_dir.mkdir()
            (repo_root / "docs" / "wiki").mkdir(parents=True)
            (wiki_dir / "Overview.md").write_text(
                "[Readme](../../README.md)",
                encoding="utf-8",
            )

            rewriter.rewrite_wiki_links(
                wiki_dir=wiki_dir,
                repository="mrdav30/FixedMathSharp",
                sync_sha="abc123",
                source_dir="docs/wiki",
                repo_root=repo_root,
            )

            content = (wiki_dir / "Overview.md").read_text(encoding="utf-8")

        self.assertIn("[Readme](../../README.md)", content)

    def test_ignores_external_images_code_and_non_markdown_local_links(self):
        rewriter = load_rewriter()

        with tempfile.TemporaryDirectory() as temp_dir:
            root = pathlib.Path(temp_dir)
            wiki_dir = root / "wiki"
            repo_root = root / "repo"
            wiki_dir.mkdir()
            (repo_root / "docs" / "wiki").mkdir(parents=True)

            (wiki_dir / "Overview.md").write_text(
                "\n".join(
                    [
                        "[External](https://example.com/Overview.md)",
                        "![Image](Overview.md)",
                        "`[Code](Overview.md)`",
                        "```",
                        "[Fence](Overview.md)",
                        "```",
                        "[Asset](notes.txt)",
                    ]
                ),
                encoding="utf-8",
            )

            rewriter.rewrite_wiki_links(
                wiki_dir=wiki_dir,
                repository="mrdav30/FixedMathSharp",
                sync_sha="abc123",
                source_dir="docs/wiki",
                repo_root=repo_root,
            )

            content = (wiki_dir / "Overview.md").read_text(encoding="utf-8")

        self.assertIn("[External](https://example.com/Overview.md)", content)
        self.assertIn("![Image](Overview.md)", content)
        self.assertIn("`[Code](Overview.md)`", content)
        self.assertIn("[Fence](Overview.md)", content)
        self.assertIn("[Asset](notes.txt)", content)

    def test_fails_when_wiki_page_link_target_does_not_exist(self):
        rewriter = load_rewriter()

        with tempfile.TemporaryDirectory() as temp_dir:
            root = pathlib.Path(temp_dir)
            wiki_dir = root / "wiki"
            repo_root = root / "repo"
            wiki_dir.mkdir()
            (repo_root / "docs" / "wiki").mkdir(parents=True)
            (wiki_dir / "Overview.md").write_text(
                "[Missing](Missing.md)",
                encoding="utf-8",
            )

            with self.assertRaises(FileNotFoundError):
                rewriter.rewrite_wiki_links(
                    wiki_dir=wiki_dir,
                    repository="mrdav30/FixedMathSharp",
                    sync_sha="abc123",
                    source_dir="docs/wiki",
                    repo_root=repo_root,
                )


if __name__ == "__main__":
    unittest.main()
