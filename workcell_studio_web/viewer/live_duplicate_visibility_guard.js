function installLiveDuplicateVisibilityGuard() {
  const api = window.__WORKCELL_EDITOR_API_V1__;
  if (!api || api.__WORKCELL_DUPLICATE_VISIBILITY_GUARD__ === true) return Boolean(api);
  if (typeof api.duplicateItem !== 'function' || typeof api.setVisibleItemIds !== 'function') return false;

  const duplicateItem = api.duplicateItem.bind(api);
  const setVisibleItemIds = api.setVisibleItemIds.bind(api);
  const removeItem = typeof api.removeItem === 'function' ? api.removeItem.bind(api) : null;
  const pendingDuplicates = new Map();

  api.duplicateItem = (sourceId, item) => {
    const result = duplicateItem(sourceId, item);
    const source = String(sourceId || '').trim();
    const duplicate = String(item?.id || '').trim();
    if (source && duplicate) pendingDuplicates.set(duplicate, source);
    return result;
  };

  api.setVisibleItemIds = ids => {
    const visible = new Set(
      Array.isArray(ids)
        ? ids.map(id => String(id || '').trim()).filter(Boolean)
        : []
    );

    // Qt can still have one pre-duplicate visibility update queued when the
    // browser has already created the copy.  Inherit the source item's current
    // visibility until the next authoritative mask contains the duplicate.
    // This prevents a successful live duplicate from immediately becoming an
    // invisible selected object with only its gizmo left on screen.
    for (const [duplicateId, sourceId] of [...pendingDuplicates.entries()]) {
      if (visible.has(duplicateId)) {
        pendingDuplicates.delete(duplicateId);
        continue;
      }
      if (visible.has(sourceId)) visible.add(duplicateId);
      else pendingDuplicates.delete(duplicateId);
    }

    return setVisibleItemIds([...visible]);
  };

  if (removeItem) {
    api.removeItem = id => {
      const stableId = String(id || '').trim();
      pendingDuplicates.delete(stableId);
      for (const [duplicateId, sourceId] of [...pendingDuplicates.entries()]) {
        if (sourceId === stableId) pendingDuplicates.delete(duplicateId);
      }
      return removeItem(id);
    };
  }

  Object.defineProperty(api, '__WORKCELL_DUPLICATE_VISIBILITY_GUARD__', {
    value: true,
    enumerable: false,
    configurable: false,
    writable: false,
  });
  return true;
}

if (!installLiveDuplicateVisibilityGuard()) {
  window.addEventListener('workcell:editor-api-ready', installLiveDuplicateVisibilityGuard, { once: true });
}
