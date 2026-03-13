'use strict';
/**
 * hook-handler.cjs
 * Generic hook dispatcher for claude-flow lifecycle events.
 * All handlers exit 0 (non-blocking) on error.
 */

const { execSync } = require('child_process');
const command = process.argv[2];

function runRuflo(...args) {
  try {
    execSync('ruflo ' + args.join(' '), { timeout: 4000, stdio: 'ignore' });
  } catch {
    // Not fatal
  }
}

switch (command) {
  case 'pre-bash':
  case 'post-bash':
  case 'pre-edit':
  case 'post-edit':
    // Tool use hooks — currently pass-through
    break;

  case 'route':
    // UserPromptSubmit — route classification (pass-through)
    break;

  case 'session-restore':
    runRuflo('memory', 'restore', '--quiet');
    break;

  case 'session-end':
    runRuflo('memory', 'save', '--quiet');
    break;

  case 'compact-manual':
  case 'compact-auto':
    runRuflo('memory', 'compact', '--quiet');
    break;

  case 'status':
    // SubagentStart status check — pass-through
    break;

  case 'post-task':
    // SubagentStop — pass-through
    break;

  case 'notify':
    // Notification hook — pass-through
    break;

  default:
    // Unknown command — exit cleanly
    break;
}

process.exit(0);
