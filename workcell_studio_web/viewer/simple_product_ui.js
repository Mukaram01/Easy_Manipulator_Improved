const SIMPLE_UI_VERSION = 1;
const PRIMARY_INSPECTOR_ROWS = new Map([
  ['label', 'Name'],
  ['type', 'Type'],
  ['details', 'Surface'],
  ['pose xyz', 'Position'],
  ['pose rpy', 'Rotation'],
  ['editable', 'Editable'],
  ['locked', 'Locked'],
  ['support_surface_kind', 'Support'],
  ['top_surface_z_m', 'Surface height'],
  ['render_status', 'Visual'],
]);

const runtime = {
  inspectorObserver: null,
  warningObserver: null,
  toolbarObserver: null,
  installed: false,
};

function moveTaskGizmoControl() {
  const slot = document.getElementById('task-gizmo-slot');
  const input = document.getElementById('free-height-toggle');
  const label = input?.closest('label');
  if (slot && label && label.parentElement !== slot) slot.appendChild(label);
}

function consolidateToolbar() {
  const panel = document.querySelector('#more-tools .toolbar-more-panel');
  const clearEdits = document.getElementById('clear-edits');
  if (!panel) return;
  for (const control of [
    document.getElementById('undo-edit'),
    document.getElementById('redo-edit'),
    document.getElementById('transform-space')?.closest('label'),
  ]) {
    if (control && control.parentElement !== panel) panel.insertBefore(control, clearEdits || panel.firstChild);
  }
}

function firstTextNode(label) {
  return Array.from(label?.childNodes || []).find(node => node.nodeType === Node.TEXT_NODE) || null;
}

function renameTransformField(name, labelText) {
  const input = document.querySelector(`#inspector [data-transform-field="${name}"]`);
  const label = input?.closest('label');
  const text = firstTextNode(label);
  if (text) text.textContent = labelText;
}

function setTransformFieldVisible(name, visible) {
  const input = document.querySelector(`#inspector [data-transform-field="${name}"]`);
  input?.closest('label')?.classList.toggle('simple-ui-hidden-field', !visible);
}

function syncTaskTransformFields() {
  const freeHeight = Boolean(document.getElementById('free-height-toggle')?.checked);
  setTransformFieldVisible('x', true);
  setTransformFieldVisible('y', true);
  setTransformFieldVisible('z', freeHeight);
  setTransformFieldVisible('roll', false);
  setTransformFieldVisible('pitch', false);
  setTransformFieldVisible('yaw', true);
  setTransformFieldVisible('scale_x', false);
  setTransformFieldVisible('scale_y', false);
  setTransformFieldVisible('scale_z', false);
}

function simplifyTransformEditor(editor) {
  if (!editor) return;
  const title = editor.querySelector('h3');
  if (title) title.textContent = 'Place item';

  const note = editor.querySelector('.edit-note');
  if (note) {
    note.textContent = 'Move on the surface, turn around Z and hold Shift for fine adjustment. Export edits when finished.';
  }

  renameTransformField('x', 'Left / right (X)');
  renameTransformField('y', 'Forward / back (Y)');
  renameTransformField('z', 'Height (Z)');
  renameTransformField('yaw', 'Turn (Yaw)');
  syncTaskTransformFields();
}

function buildTechnicalDetails(rows) {
  if (!rows.length) return null;
  const details = document.createElement('details');
  details.className = 'technical-details';
  const summary = document.createElement('summary');
  summary.textContent = `Technical details (${rows.length})`;
  const table = document.createElement('table');
  table.className = 'inspector-table';
  const body = document.createElement('tbody');
  rows.forEach(row => body.appendChild(row));
  table.appendChild(body);
  details.append(summary, table);
  return details;
}

function simplifyInspector() {
  const inspector = document.getElementById('inspector');
  if (!inspector) return;
  const table = inspector.querySelector(':scope > .inspector-table');
  if (!table || table.dataset.simpleProductUi === String(SIMPLE_UI_VERSION)) {
    simplifyTransformEditor(inspector.querySelector('.transform-editor'));
    return;
  }

  const technicalRows = [];
  table.querySelectorAll('tbody > tr').forEach(row => {
    const heading = row.querySelector('th');
    const key = String(heading?.textContent || '').trim().toLowerCase();
    const friendly = PRIMARY_INSPECTOR_ROWS.get(key);
    if (friendly) heading.textContent = friendly;
    else technicalRows.push(row);
  });
  table.dataset.simpleProductUi = String(SIMPLE_UI_VERSION);

  const transformEditor = inspector.querySelector('.transform-editor');
  const technical = buildTechnicalDetails(technicalRows);
  if (technical) inspector.insertBefore(technical, transformEditor || null);
  simplifyTransformEditor(transformEditor);
}

function updateWarningDisclosure() {
  const details = document.getElementById('warnings-details');
  const warnings = document.getElementById('warnings');
  const countNode = document.getElementById('warning-count');
  if (!details || !warnings || !countNode) return;

  const items = warnings.querySelectorAll('.warning-item').length;
  const count = items || (warnings.classList.contains('empty') ? 0 : 1);
  countNode.textContent = String(count);
  details.hidden = count === 0;
  if (count > 0) details.open = true;
}

function closeMoreTools(event) {
  const more = document.getElementById('more-tools');
  if (!more?.open) return;
  if (event.type === 'keydown' && event.key === 'Escape') {
    more.open = false;
    return;
  }
  if (event.type === 'pointerdown' && !more.contains(event.target)) more.open = false;
}

function installObservers() {
  const inspector = document.getElementById('inspector');
  if (inspector && !runtime.inspectorObserver) {
    runtime.inspectorObserver = new MutationObserver(simplifyInspector);
    runtime.inspectorObserver.observe(inspector, { childList: true, subtree: true });
  }

  const warnings = document.getElementById('warnings');
  if (warnings && !runtime.warningObserver) {
    runtime.warningObserver = new MutationObserver(updateWarningDisclosure);
    runtime.warningObserver.observe(warnings, { childList: true, subtree: true, attributes: true, attributeFilter: ['class'] });
  }

  const toolbar = document.querySelector('.toolbar');
  if (toolbar && !runtime.toolbarObserver) {
    runtime.toolbarObserver = new MutationObserver(moveTaskGizmoControl);
    runtime.toolbarObserver.observe(toolbar, { childList: true, subtree: true });
  }
}

function install() {
  if (runtime.installed) return;
  runtime.installed = true;
  document.body.classList.add('simple-product-ui');
  installObservers();
  consolidateToolbar();
  moveTaskGizmoControl();
  simplifyInspector();
  updateWarningDisclosure();

  document.addEventListener('change', event => {
    if (event.target?.id === 'free-height-toggle') syncTaskTransformFields();
  });
  document.addEventListener('pointerdown', closeMoreTools);
  document.addEventListener('keydown', closeMoreTools);

  window.__WORKCELL_SIMPLE_PRODUCT_UI_V1__ = Object.freeze({
    enabled: true,
    version: SIMPLE_UI_VERSION,
    refresh: () => {
      moveTaskGizmoControl();
      consolidateToolbar();
      simplifyInspector();
      updateWarningDisclosure();
    },
  });
}

install();
