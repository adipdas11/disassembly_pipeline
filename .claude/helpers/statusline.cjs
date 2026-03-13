'use strict';
/**
 * statusline.cjs
 * Outputs a status line for Claude Code's status bar.
 */

const { execSync } = require('child_process');

let status = 'claude-flow';

try {
  const result = execSync('ruflo mcp status --json 2>/dev/null', { timeout: 2000 }).toString().trim();
  const data = JSON.parse(result);
  if (data && data.status === 'Running') {
    status = 'ruflo:running';
  }
} catch {
  // Not available — use default
}

process.stdout.write(status + '\n');
process.exit(0);
