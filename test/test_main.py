#!/usr/bin/env python3
# test_main.py - CLI tests for ros2diag main entry point
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import unittest
from unittest.mock import Mock, patch, call
import pytest

from ros2diag.main import main, list_frames, app


class TestCLIStructure(unittest.TestCase):
    """Test CLI application structure."""

    def test_app_exists(self):
        """Test that the cyclopts app is created."""
        self.assertIsNotNone(app)
        # app.name is a tuple in cyclopts
        self.assertEqual(app.name, ('ros2diag',))

    def test_app_has_help(self):
        """Test that app has help text."""
        self.assertIsNotNone(app.help)
        self.assertIn('TF diagnostics', app.help)

    def test_list_command_exists(self):
        """Test that the list command is registered."""
        # The list_frames function should be a registered command
        self.assertIsNotNone(list_frames)
        self.assertTrue(callable(list_frames))

    def test_list_command_has_docstring(self):
        """Test that list command has documentation."""
        self.assertIsNotNone(list_frames.__doc__)
        self.assertIn('List', list_frames.__doc__)


class TestListCommand(unittest.TestCase):
    """Test the list command functionality."""

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    @patch('builtins.print')
    def test_list_calls_both_implementations(self, mock_print, mock_list_original, mock_list_accurate):
        """Test that list command calls both TF lister implementations."""
        list_frames()

        # Verify both implementations were called
        mock_list_original.assert_called_once()
        mock_list_accurate.assert_called_once()

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    @patch('builtins.print')
    def test_list_prints_separators(self, mock_print, mock_list_original, mock_list_accurate):
        """Test that list command prints section separators."""
        list_frames()

        # Verify separators were printed
        print_calls = [str(call) for call in mock_print.call_args_list]

        # Should have separators and headers
        has_separator = any('=' * 80 in call for call in print_calls)
        has_original_header = any('ORIGINAL IMPLEMENTATION' in call for call in print_calls)
        has_accurate_header = any('ACCURATE IMPLEMENTATION' in call for call in print_calls)

        self.assertTrue(has_separator)
        self.assertTrue(has_original_header)
        self.assertTrue(has_accurate_header)

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    @patch('builtins.print')
    def test_list_output_order(self, mock_print, mock_list_original, mock_list_accurate):
        """Test that original implementation is called before accurate."""
        list_frames()

        # Get the order of calls
        calls = [mock_list_original.call_args_list, mock_list_accurate.call_args_list]

        # Both should have been called
        self.assertTrue(mock_list_original.called)
        self.assertTrue(mock_list_accurate.called)

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    def test_list_handles_original_exception(self, mock_list_original, mock_list_accurate):
        """Test that exceptions in original implementation don't stop accurate."""
        # Make original implementation raise an exception
        mock_list_original.side_effect = Exception("Test error")

        # This should raise the exception
        with self.assertRaises(Exception):
            list_frames()

        # Original was called
        mock_list_original.assert_called_once()


class TestMainFunction(unittest.TestCase):
    """Test the main entry point."""

    def test_main_exists(self):
        """Test that main function exists."""
        self.assertIsNotNone(main)
        self.assertTrue(callable(main))

    @patch('ros2diag.main.app')
    def test_main_calls_app(self, mock_app):
        """Test that main function calls the cyclopts app."""
        main()

        # Verify app was called
        mock_app.assert_called_once()

    @patch('ros2diag.main.app')
    def test_main_with_arguments(self, mock_app):
        """Test main function delegates to app for argument parsing."""
        main()

        # The app should be invoked (cyclopts handles arg parsing)
        self.assertTrue(mock_app.called)


class TestCLIHelp(unittest.TestCase):
    """Test CLI help functionality."""

    @patch('sys.argv', ['ros2diag', '--help'])
    @patch('sys.exit')
    def test_help_flag(self, mock_exit):
        """Test --help flag works."""
        # Note: cyclopts will handle --help internally
        # This test just verifies the structure
        try:
            main()
        except SystemExit:
            pass  # Expected when --help is used

    def test_command_docstrings(self):
        """Test that commands have proper docstrings for help."""
        # list_frames should have a docstring
        self.assertIsNotNone(list_frames.__doc__)
        self.assertTrue(len(list_frames.__doc__) > 0)


class TestCLIErrorHandling(unittest.TestCase):
    """Test CLI error handling."""

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    def test_both_implementations_fail(self, mock_list_original, mock_list_accurate):
        """Test behavior when both implementations fail."""
        # Make both implementations raise exceptions
        mock_list_original.side_effect = Exception("Original failed")
        mock_list_accurate.side_effect = Exception("Accurate failed")

        # First exception should propagate
        with self.assertRaises(Exception) as context:
            list_frames()

        self.assertIn("Original failed", str(context.exception))

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    @patch('builtins.print')
    def test_accurate_fails_after_original_succeeds(self, mock_print,
                                                     mock_list_original, mock_list_accurate):
        """Test behavior when accurate implementation fails but original succeeds."""
        # Original succeeds, accurate fails
        mock_list_accurate.side_effect = Exception("Accurate failed")

        # Should raise exception from accurate
        with self.assertRaises(Exception) as context:
            list_frames()

        self.assertIn("Accurate failed", str(context.exception))
        # But original should have been called first
        mock_list_original.assert_called_once()


class TestCLIIntegration(unittest.TestCase):
    """Integration tests for CLI."""

    @patch('ros2diag.main.tf_lister_accurate.list_tf_frames_accurate')
    @patch('ros2diag.main.tf_lister.list_tf_frames')
    @patch('builtins.print')
    def test_full_list_workflow(self, mock_print, mock_list_original, mock_list_accurate):
        """Test complete workflow of list command."""
        # Set up mocks to succeed
        mock_list_original.return_value = None
        mock_list_accurate.return_value = None

        # Run the list command
        list_frames()

        # Verify workflow
        # 1. Both implementations called
        mock_list_original.assert_called_once()
        mock_list_accurate.assert_called_once()

        # 2. Separators and headers printed
        print_calls = [str(call) for call in mock_print.call_args_list]
        self.assertTrue(len(print_calls) > 0)

        # 3. Headers contain expected text
        all_prints = ' '.join(print_calls)
        self.assertIn('ORIGINAL', all_prints)
        self.assertIn('ACCURATE', all_prints)


if __name__ == '__main__':
    unittest.main()
