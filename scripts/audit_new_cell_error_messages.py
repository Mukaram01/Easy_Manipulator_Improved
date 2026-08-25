#!/usr/bin/env python3
from __future__ import annotations
import json
from workcell_studio_error_messages import MESSAGES, STANDARD_FIELDS

CHECK_CODES=['MISSING_WORKSPACE','INVALID_SCENE_NAME','MISSING_SAVED_LAYOUT','MISSING_ENVIRONMENT_LAYOUT','MISSING_TASK_INTENT','MISSING_PACKAGE_XML','MISSING_DEMO_LAUNCH','MISSING_FAKE_HARDWARE_ARG','VALIDATION_BLOCKED']

vague={'failed','missing file','invalid'}
missing=[c for c in CHECK_CODES if c not in MESSAGES]
actionable=[c for c,m in MESSAGES.items() if m.get('next_action') and m.get('detail') and m.get('why_it_matters')]
vague_hits=[c for c,m in MESSAGES.items() if any(v in (m.get('title','')+' '+m.get('detail','')).lower() for v in vague)]
status='PASS' if not missing else 'BLOCKED'
if status=='PASS' and vague_hits: status='WARNINGS'
report={'checked_errors':CHECK_CODES,'missing_message_codes':missing,'vague_messages':vague_hits,'actionable_messages':actionable,'blocker_message_status':status,'required_fields':STANDARD_FIELDS}
print(json.dumps(report,indent=2))
raise SystemExit(0 if status=='PASS' else 1)
