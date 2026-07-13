import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createHash } from 'node:crypto';
import { build } from 'esbuild';

const repoViewerRoot = new URL('..', import.meta.url);
const entryPoint = new URL('../src/viewer_entry.js', import.meta.url).pathname;
const committedBundle = new URL('../dist/viewer.bundle.js', import.meta.url).pathname;
const tmpRoot = await mkdtemp(join(tmpdir(), 'workcell-web3d-bundle-'));
const tmpBundle = join(tmpRoot, 'viewer.bundle.js');

function sha256(buffer) {
  return createHash('sha256').update(buffer).digest('hex');
}

try {
  await build({
    entryPoints: [entryPoint],
    bundle: true,
    format: 'esm',
    target: 'es2020',
    sourcemap: true,
    outfile: tmpBundle,
    absWorkingDir: repoViewerRoot.pathname,
  });

  const [expected, actual] = await Promise.all([
    readFile(tmpBundle),
    readFile(committedBundle),
  ]);
  const expectedHash = sha256(expected);
  const actualHash = sha256(actual);
  if (expectedHash !== actualHash) {
    console.error('workcell_studio_web/viewer/dist/viewer.bundle.js is stale.');
    console.error(`rebuilt sha256:   ${expectedHash}`);
    console.error(`committed sha256: ${actualHash}`);
    console.error('Run: npm ci && npm run build:web3d');
    process.exit(1);
  }
  console.log(`viewer.bundle.js is current (${actualHash}).`);
} finally {
  await rm(tmpRoot, { recursive: true, force: true });
}
