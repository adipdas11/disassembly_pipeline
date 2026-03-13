#!/usr/bin/env node
/**
 * auto-memory-hook.mjs
 * Handles memory sync/import for claude-flow sessions.
 * Commands: sync (Stop hook), import (SessionStart hook)
 */

import { execSync } from 'child_process';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'fs';
import { join } from 'path';

const command = process.argv[2];
const projectDir = process.env.CLAUDE_PROJECT_DIR || process.cwd();
const memoryDir = join(projectDir, '.claude', 'memory');

function ensureMemoryDir() {
  if (!existsSync(memoryDir)) {
    mkdirSync(memoryDir, { recursive: true });
  }
}

function syncMemory() {
  try {
    ensureMemoryDir();
    // Attempt to sync via ruflo if available
    try {
      execSync('ruflo memory export --format json --output "' + join(memoryDir, 'session-memory.json') + '"', {
        timeout: 5000,
        stdio: 'ignore'
      });
    } catch {
      // ruflo memory export not available or no data — not fatal
    }
  } catch (err) {
    // Non-fatal: memory sync failure should not block session stop
    process.stderr.write('auto-memory-hook sync warning: ' + err.message + '\n');
  }
}

function importMemory() {
  try {
    ensureMemoryDir();
    const memFile = join(memoryDir, 'session-memory.json');
    if (!existsSync(memFile)) return;
    try {
      execSync('ruflo memory import --format json --input "' + memFile + '"', {
        timeout: 5000,
        stdio: 'ignore'
      });
    } catch {
      // ruflo memory import not available — not fatal
    }
  } catch (err) {
    process.stderr.write('auto-memory-hook import warning: ' + err.message + '\n');
  }
}

switch (command) {
  case 'sync':
    syncMemory();
    break;
  case 'import':
    importMemory();
    break;
  default:
    // Unknown command — exit cleanly
    break;
}

process.exit(0);
