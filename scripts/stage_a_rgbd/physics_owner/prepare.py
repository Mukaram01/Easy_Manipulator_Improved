#!/usr/bin/env python3
"""Fetch hash-pinned Fortress source and add read-only owner instrumentation.

All outputs are an opt-in build overlay; nothing is installed or overwritten.
"""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

HERE=Path(__file__).resolve().parent

def once(text,old,new):
    if text.count(old)!=1:raise ValueError('source anchor changed: '+old[:100])
    return text.replace(old,new)

def reporting_reference(cc):
    cc=once(cc,'  for (const auto &contactComposite : allContacts)\n',
        '  std::unordered_map<const WorldShapeType::ContactPoint *, const WorldShapeType::ExtraContactData *> reportExtras;\n\n  for (const auto &contactComposite : allContacts)\n')
    cc=once(cc,'    auto coll1Entity =\n',
        '    reportExtras.emplace(&contact, contactComposite.Query<WorldShapeType::ExtraContactData>());\n    auto coll1Entity =\n')
    anchor='            position->set_z(contact->point.z());'
    cc=once(cc,anchor,anchor+"""
            const auto extra = reportExtras.at(contact);
            if (extra)
            {
              // Reporting normal acts on the reported first body (2 -> 1).
              const auto first = this->entityCollisionMap.GetByPhysicsId(
                  contact->collision1->EntityID());
              const double sign = first == _collEntity1 ? 1.0 : -1.0;
              auto *normal = contactMsg->add_normal();
              normal->set_x(sign * extra->normal.x());
              normal->set_y(sign * extra->normal.y());
              normal->set_z(sign * extra->normal.z());
              contactMsg->add_depth(extra->depth);
            }""")
    return cc


def rename(cc,hh,name):
    for old,new in [('PhysicsPrivate',name+'Private'),('Physics::',name+'::'),('class Physics:','class '+name+':'),('explicit Physics();','explicit '+name+'();'),('~Physics()','~'+name+'()'),('Physics() :',name+'() :'),('PLUGIN(Physics,','PLUGIN('+name+','),('ALIAS(Physics,','ALIAS('+name+','),('systems::Physics"','systems::'+name+'"')]:
        cc=cc.replace(old,new);hh=hh.replace(old,new)
    return cc,hh


def prepare(output,reference=False):
    output.mkdir(parents=True,exist_ok=True)
    pristine=output/"upstream";pristine.mkdir(exist_ok=True)
    manifest=json.loads((HERE/'source_manifest.json').read_text())
    for name,item in manifest.items():
        cached=pristine/name
        data=cached.read_bytes() if cached.exists() else urllib.request.urlopen(item['url'],timeout=30).read()
        if hashlib.sha256(data).hexdigest()!=item['sha256']:raise ValueError('source hash mismatch: '+name)
        cached.write_bytes(data)
        (output/name).write_bytes(data)
    cc=(output/'Physics.cc').read_text();hh=(output/'Physics.hh').read_text()
    if reference:
        cc,hh=rename(reporting_reference(cc),hh,'WorkcellReferencePhysics')
        (output/'Physics.cc').write_text(cc);(output/'Physics.hh').write_text(hh)
        (output/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        return
    cc=once(cc,'#include "EntityFeatureMap.hh"','#include "EntityFeatureMap.hh"\n#include "owner_readback.hh"')
    cc=once(cc,'  public: bool contactsEntityNames = true;','  public: bool contactsEntityNames = true;\n#include "owner_private.inc"')
    cc=once(cc,'          SolverFeatureList>;','          SolverFeatureList, OwnerWorldFeatures>;')
    cc=once(cc,'  /// \\brief World EntityFeatureMap','  public: struct OwnerWorldFeatures : physics::FeatureList<MinimumFeatureList, physics::dartsim::RetrieveWorld>{};\n\n  /// \\brief World EntityFeatureMap')
    cc=once(cc,'        ShapePtrType collisionPtrPhys;','        const auto ownerBefore = this->OwnerNodes();\n        ShapePtrType collisionPtrPhys;')
    cc=once(cc,'        this->entityCollisionMap.AddEntity(_entity, collisionPtrPhys);','        this->entityCollisionMap.AddEntity(_entity, collisionPtrPhys);\n        this->owner.bindings.Bind(_entity, collisionPtrPhys->EntityID(), ownerBefore, this->OwnerNodes());')
    cc=once(cc,'      break;\n    }\n\n    auto missingFeatures', '      this->dataPtr->owner.backendPath = pathToLib;\n      this->dataPtr->owner.backendClass = className;\n      break;\n    }\n\n    auto missingFeatures')
    cc=once(cc,'  this->dataPtr->eventManager = &_eventMgr;','  this->dataPtr->eventManager = &_eventMgr;\n  this->dataPtr->owner.Configure(_sdf);')
    cc=once(cc,'      stepOutput = this->dataPtr->Step(_info.dt);','      this->dataPtr->OwnerBeforeStep(_info);\n      stepOutput = this->dataPtr->Step(_info.dt);')
    cc=once(cc,'    this->dataPtr->RemovePhysicsEntities(_ecm);','    this->dataPtr->RemovePhysicsEntities(_ecm);\n    if (!_info.paused) this->dataPtr->OwnerRecord(_info, _ecm);')
    cc,hh=rename(cc,hh,'WorkcellOwnerPhysics')
    (output/'Physics.cc').write_text(cc);(output/'Physics.hh').write_text(hh)
    (output/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path);parser.add_argument('--reference',action='store_true')
    args=parser.parse_args();prepare(args.output,args.reference)
