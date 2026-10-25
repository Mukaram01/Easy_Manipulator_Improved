import importlib.util
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))

import pytest


SCRIPT=Path(__file__).parents[1]/'scripts/build_ros_gz_bridge_shutdown_overlay.py'
SPEC=importlib.util.spec_from_file_location('bridge_overlay_builder',SCRIPT)
MODULE=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def source_fixture(tmp_path):
    source=tmp_path/'source';source.mkdir()
    def git(*args):return subprocess.check_output(['git','-C',str(source),*args])
    git('init','-q')
    (source/'main.cpp').write_text('spin();\nreturn 0;\n')
    (source/'header.hpp').write_text('reviewed header\n')
    git('add','.');git('-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','fixture')
    (source/'main.cpp').write_text('spin();\nshutdown();\nreturn 0;\n')
    patch=tmp_path/'reviewed.patch';patch.write_bytes(git('diff','--binary'))
    git('restore','main.cpp')
    return source,patch,git


def test_bridge_source_guard_accepts_only_reviewed_patch_and_is_repeatable(tmp_path):
    source,patch,_=source_fixture(tmp_path)
    assert MODULE.apply_reviewed_patch(source,patch)==patch.read_bytes()
    assert MODULE.apply_reviewed_patch(source,patch)==patch.read_bytes()


@pytest.mark.parametrize('staged',[False,True])
def test_bridge_source_guard_rejects_extra_changes_even_in_index(tmp_path,staged):
    source,patch,git=source_fixture(tmp_path)
    (source/'header.hpp').write_text('unreviewed source change\n')
    if staged:git('add','header.hpp')
    with pytest.raises(RuntimeError,match='beyond the reviewed patch'):
        MODULE.apply_reviewed_patch(source,patch)


def test_unrelated_shutdown_exception_cannot_count_as_lifetime_baseline():
    spec=importlib.util.spec_from_file_location('bridge_verify',SCRIPT.parent/'bridge_shutdown_repro/verify.py')
    verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
    assert not verifier.baseline_lifetime_abort([0,-6],"terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'")
    assert not verifier.baseline_lifetime_abort([0,-6],'unclassified abort')
    assert verifier.baseline_lifetime_abort([0,-6],'free(): corrupted unsorted chunks\n')
    assert not verifier.baseline_lifetime_abort([0,0],'free(): corrupted unsorted chunks\n')


def legacy_admission_fixture(tmp_path):
    """Synthetic files test admission only; they are never runtime evidence."""
    import hashlib
    import json
    def identity(path, text):
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
        return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    patch=identity(tmp_path/'patch','synthetic reviewed patch')
    executable=identity(tmp_path/'install/lib/ros_gz_bridge/parameter_bridge','synthetic executable')
    library=identity(tmp_path/'libros_gz_bridge.so','synthetic library')
    result=identity(tmp_path/'results.json','synthetic legacy proof')
    summary=dict(cycles=20,passed=20,errors=0,results_path=result['path'],results_sha256=result['sha256'])
    manifest=dict(schema='workcell_ros_gz_bridge_shutdown/v1',source=dict(version='0.244.26',commit=MODULE.SOURCE_COMMIT),
        patch=patch,executable=executable,library=library,
        baseline=dict(package='ros-humble-ros-gz-bridge',version='0.244.26-1jammy.test'),
        reproduction=dict(baseline_returncode=-6,native=summary,memcheck=dict(summary,cycles=5,passed=5)))
    path=tmp_path/'provenance.json';path.write_text(json.dumps(manifest))
    return path,Path(patch['path']),manifest


def test_legacy_admission_is_unchanged_with_absent_or_explicit_legacy_method(tmp_path):
    import json
    from simulator_backend import read_bridge_overlay
    path,patch,data=legacy_admission_fixture(tmp_path)
    assert read_bridge_overlay(path,patch)['reproduction']['baseline_returncode']==-6
    data['qualification']=dict(method='legacy-installed-baseline/v1')
    path.write_text(json.dumps(data))
    assert read_bridge_overlay(path,patch)['reproduction']['baseline_returncode']==-6


@pytest.mark.parametrize('qualification',[
    {}, {'method':'unknown/v1'}, {'method':None},
    {'method':'deterministic-source-lifetime/v1'},
    {'evidence':{'path':'missing','sha256':'0'*64}},
])
def test_alternative_admission_cannot_reuse_legacy_summary_without_evidence(tmp_path,qualification):
    import json
    from simulator_backend import read_bridge_overlay
    path,patch,data=legacy_admission_fixture(tmp_path)
    data['qualification']=qualification
    path.write_text(json.dumps(data))
    with pytest.raises(RuntimeError,match='BRIDGE_OVERLAY_REJECTED'):
        read_bridge_overlay(path,patch)


def test_runtime_validation_rejects_legacy_summary_only(tmp_path):
    from bridge_shutdown_repro import qualification
    _,_,data=legacy_admission_fixture(tmp_path)
    with pytest.raises(ValueError,match='runtime'):
        qualification.validate_runtime(data)


def test_publication_removes_old_discovery_when_evidence_is_incomplete(tmp_path):
    from bridge_shutdown_repro import qualification
    root=tmp_path/'overlay';root.mkdir()
    (root/'provenance.json').write_text('stale publication')
    (root/'setup.bash').write_text('stale setup')
    with pytest.raises((ValueError,RuntimeError)):
        qualification.publish(root,{},tmp_path/'missing.patch')
    assert not (root/'provenance.json').exists()
    assert not (root/'setup.bash').exists()


def test_builder_requires_explicit_method_and_both_alternative_records():
    parser=MODULE.argument_parser()
    with pytest.raises(SystemExit):parser.parse_args([])          # no machine-specific default overlay root
    assert parser.parse_args(['--output','o']).qualification_method=='legacy-installed-baseline/v1'
    for arguments in (['--build-only','--qualification-method','observed-shutdown-order/v1'],
                      ['--build-only','--mechanism-review','review.json']):
        with pytest.raises(SystemExit):
            MODULE.validate_arguments(parser,parser.parse_args(['--output','o',*arguments]))
    with pytest.raises(SystemExit):
        parser.parse_args(['--output','o','--qualification-method','unknown/v1'])
    for arguments in (['--lifetime-evidence','record.json'],
                      ['--qualification-method','deterministic-source-lifetime/v1'],
                      ['--qualification-method','deterministic-source-lifetime/v1',
                       '--lifetime-evidence','record.json']):
        with pytest.raises(SystemExit):
            MODULE.validate_arguments(parser,parser.parse_args(['--output','o',*arguments]))


def lifetime_admission_fixture(tmp_path):
    """Synthetic admission fixture: NOT a causal experiment or runtime qualification."""
    import hashlib
    import json
    import time
    from bridge_shutdown_repro import qualification as q
    path,patch,manifest=legacy_admission_fixture(tmp_path)
    def artifact(name,text):
        file=tmp_path/name;file.parent.mkdir(parents=True,exist_ok=True);file.write_text(text)
        return dict(path=str(file),sha256=hashlib.sha256(file.read_bytes()).hexdigest())
    def json_artifact(name,data):
        return artifact(name,json.dumps(data))
    def mapping(records):
        return ''.join('1000-2000 r-xp 00000000 00:01 '+str(Path(item['path']).stat().st_ino)+' '+item['path']+'\n'
                       for item in records)
    source_main=artifact('sources/parameter_bridge.cpp','synthetic runtime source')
    manifest['source']['patched_main_sha256']=source_main['sha256']
    manifest['reproduction'].pop('baseline_returncode')
    manifest['runtime_started_ns']=time.time_ns()-1_000_000_000
    manifest['runtime_environment']=dict(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_LOCALHOST_ONLY='1',
        LD_PRELOAD='',LD_AUDIT='',VALGRIND_LIB='',VALGRIND_OPTS='')
    dependencies=[artifact('dependencies/lib'+name+'.so','synthetic '+name)
                  for name in ('rclcpp','rcl','rcutils','rcl_logging_spdlog','spdlog')]
    source=dict(bridge_commit=MODULE.SOURCE_COMMIT,files=[source_main],dependencies={
        name:dict(revision='synthetic-pinned-revision',files=[artifact('sources/'+name+'.cpp','synthetic source '+name)])
        for name in ('rclcpp','rcl','rcutils','rcl_logging_spdlog','spdlog')})
    harness=dict(files=[artifact('harness.cpp','synthetic harness')],
        instrumentation=[artifact('instrumentation.patch','synthetic schedule instrumentation')],
        instrumentation_notes=artifact('instrumentation.md','Synthetic notes: these test files are never qualification evidence.'),
        lifetime_patch=artifact('lifetime.patch','synthetic lifetime-only delta'),
        build_config=artifact('build.json','synthetic compiler and matching before/after flags'))
    evidence=dict(schema='workcell_bridge_lifetime_evidence/v1',method='deterministic-source-lifetime/v1',
        producer='synthetic-fixture-author',source=source,harness=harness,patch=manifest['patch'],
        finite_bound_seconds=30,runtime=dict(executable=manifest['executable'],library=manifest['library'],
        source_main=source_main,dependencies=dependencies))
    for name,pid in (('before',301),('after',302)):
        executable=artifact(name+'/harness','synthetic '+name+' executable')
        events=[dict(seq=index,pid=pid,tid=tid,kind=kind,object='synthetic-object-address',
                     stack=['synthetic::'+kind,'synthetic::caller']) for index,(kind,tid) in enumerate(
            [('object_destroyed',pid),('object_access',pid+10)] if name=='before' else
            [('object_access',pid+10),('signal_thread_stopped',pid),('object_destroyed',pid)],1)]
        evidence[name]=dict(executable=executable,loaded_libraries=dependencies,
            maps=artifact(name+'/maps',mapping([executable,*dependencies])),
            trace=artifact(name+'/events.jsonl',''.join(json.dumps(event)+'\n' for event in events)),
            stdout=artifact(name+'/stdout.log','synthetic captured process output'),
            command=[executable['path'],'--synthetic-test'],pid=pid,returncode=99 if name=='before' else 0,
            timed_out=False,exercised=True,elapsed_seconds=.5,
            classification='lifetime-use-after-destruction' if name=='before' else 'lifetime-ordering-prevented')
    evidence_record=json_artifact('lifetime-evidence.json',evidence)
    implementation=[q.identity(SCRIPT),q.identity(SCRIPT.parent/'simulator_backend.py'),
        q.identity(SCRIPT.parent/'bridge_shutdown_repro/qualification.py'),
        q.identity(SCRIPT.parent/'bridge_shutdown_repro/verify.py'),
        q.identity(SCRIPT.parent/'bridge_shutdown_repro/CMakeLists.txt'),
        q.identity(SCRIPT.parent/'bridge_shutdown_repro/publisher.cpp'),
        q.identity(SCRIPT.parent/'bridge_shutdown_repro/bridges.yaml'),
        q.identity(SCRIPT.parent/'bridge_shutdown_repro/shutdown_context_test.cpp'),
        q.identity(Path(__file__).resolve()),manifest['patch'],*harness['files'],*harness['instrumentation'],
        harness['instrumentation_notes'],harness['lifetime_patch'],harness['build_config']]
    review=dict(schema='workcell_bridge_lifetime_review/v1',disposition='accepted',reviewer='synthetic-independent-reviewer',
        reviewed_at_ns=manifest['runtime_started_ns']-1, evidence_sha256=evidence_record['sha256'],implementation=implementation,
        report=artifact('review.md','SYNTHETIC TEST REVIEW. No experiment was run. This fixture tests the admission parser. '
                        'Its narrative is not independent causal evidence and must never be published as a real review. '*3),
        assessment={key:'Synthetic parser exercise, not a statement of observed causal adequacy.' for key in
                    ('causal_adequacy','scheduling','correction','runtime_isolation')},
        validation=dict(source_inspection='Synthetic fixture only.',independently_executed_tests=[],inspected_test_logs=[]))
    manifest['qualification']=dict(method='deterministic-source-lifetime/v1',evidence=evidence_record,
        review=json_artifact('mechanism-review.json',review),runtime_dependencies=dependencies)
    for name,count,pid_base in (('native',20,1000),('memcheck',5,2000)):
        rows=[];artifacts=[]
        for cycle in range(1,count+1):
            for suffix,text in [('bridge.log','synthetic clean shutdown'),('publisher.log','synthetic publisher'),
                                ('bridge.maps',mapping([manifest['executable'],manifest['library'],*dependencies]))]:
                artifacts.append(artifact(name+f'/{cycle:02}-{suffix}',text))
            if name=='memcheck':artifacts.append(artifact(name+f'/{cycle:02}-memcheck.log','ERROR SUMMARY: 0 errors'))
            rows.append(dict(cycle=cycle,passed=True,received=300,returncodes=[0,0],
                loaded_libraries=[manifest['library']['path']],mapped_executable=True,
                loaded_executable=manifest['executable']['path'],memcheck_errors=0,core_inventory_changed=False,
                remaining_processes=[],pids=[pid_base+cycle*2,pid_base+cycle*2+1]))
        result=dict(cycles=count,completed=count,passed=count,errors=0,all_success=True,expected_abort=False,
            binary=manifest['executable']['path'],binary_sha256=manifest['executable']['sha256'],
            library=manifest['library']['path'],library_sha256=manifest['library']['sha256'],results=rows)
        record=json_artifact(name+'/results.json',result)
        manifest['reproduction'][name]=dict(cycles=count,passed=count,errors=0,
            results_path=record['path'],results_sha256=record['sha256'],artifacts=artifacts)
    for field,name in (('build_script_sha256','build_ros_gz_bridge_shutdown_overlay.py'),
                       ('proof_script_sha256','bridge_shutdown_repro/verify.py'),
                       ('publisher_source_sha256','bridge_shutdown_repro/publisher.cpp'),
                       ('configuration_sha256','bridge_shutdown_repro/bridges.yaml')):
        manifest[field]=q.identity(SCRIPT.parent/name)['sha256']
    context_log=artifact('shutdown-context.log','SYNTHETIC context test output, not actual execution evidence')
    manifest['shutdown_context_test']=dict(source_sha256=q.identity(SCRIPT.parent/'bridge_shutdown_repro/shutdown_context_test.cpp')['sha256'],
        log_path=context_log['path'],log_sha256=context_log['sha256'])
    path.write_text(json.dumps(manifest))
    return path,patch,manifest,json.loads(json.dumps(evidence)),json.loads(json.dumps(review))


def save_bound_record(manifest,field,data):
    import json
    from bridge_shutdown_repro.qualification import identity
    path=Path(manifest['qualification'][field]['path']);path.write_text(json.dumps(data))
    manifest['qualification'][field]=identity(path)


def test_source_lifetime_admission_accepts_complete_bound_synthetic_record(tmp_path):
    from simulator_backend import read_bridge_overlay
    path,patch,data,_,_=lifetime_admission_fixture(tmp_path)
    admitted=read_bridge_overlay(path,patch)
    assert admitted['qualification']['method']=='deterministic-source-lifetime/v1'
    assert 'baseline_returncode' not in admitted['reproduction']
    assert 'results_path' not in admitted['baseline']


@pytest.mark.parametrize('change',[
    'missing-method','unknown-method','legacy-field','rclerror','unclassified','allocator-only','timeout','unexercised',
    'missing-review','rejected-review','review-record-mismatch','review-implementation-mismatch','boolean-review',
    'source-hash','harness-hash','runtime-hash','trace-hash','maps-hash','trace-order','patched-access-after-destruction',
    'native-incomplete','memcheck-incomplete','raw-row-failed','raw-runtime-maps','old-runtime',
])
def test_alternative_rejects_unbound_or_noncausal_evidence(tmp_path,change):
    import json
    from bridge_shutdown_repro.qualification import identity
    from simulator_backend import read_bridge_overlay
    path,patch,data,evidence,review=lifetime_admission_fixture(tmp_path)
    if change=='missing-method':data['qualification'].pop('method')
    elif change=='unknown-method':data['qualification']['method']='future/v2'
    elif change=='legacy-field':data['reproduction']['baseline_returncode']=-6
    elif change=='rclerror':
        stdout=Path(evidence['before']['stdout']['path']);stdout.write_text('rclcpp::exceptions::RCLError')
        evidence['before']['stdout']=identity(stdout)
    elif change=='unclassified':evidence['before']['classification']='unclassified-crash'
    elif change=='allocator-only':
        trace=Path(evidence['before']['trace']['path']);trace.write_text('free(): corrupted unsorted chunks\n')
        evidence['before']['trace']=identity(trace)
    elif change=='timeout':evidence['before']['timed_out']=True
    elif change=='unexercised':evidence['after']['exercised']=False
    elif change=='missing-review':data['qualification'].pop('review')
    elif change=='rejected-review':review['disposition']='rejected'
    elif change=='review-record-mismatch':review['evidence_sha256']='0'*64
    elif change=='review-implementation-mismatch':review['implementation'][0]['sha256']='0'*64
    elif change=='boolean-review':review=dict(reviewed=True)
    elif change=='source-hash':evidence['source']['files'][0]['sha256']='0'*64
    elif change=='harness-hash':evidence['harness']['files'][0]['sha256']='0'*64
    elif change=='runtime-hash':evidence['runtime']['executable']['sha256']='0'*64
    elif change=='trace-hash':evidence['before']['trace']['sha256']='0'*64
    elif change=='maps-hash':evidence['after']['maps']['sha256']='0'*64
    elif change in ('trace-order','patched-access-after-destruction'):
        phase='before' if change=='trace-order' else 'after'
        trace=Path(evidence[phase]['trace']['path']);events=[json.loads(line) for line in trace.read_text().splitlines()]
        events.reverse()
        for seq,event in enumerate(events,1):event['seq']=seq
        trace.write_text(''.join(json.dumps(event)+'\n' for event in events));evidence[phase]['trace']=identity(trace)
    elif change=='native-incomplete':data['reproduction']['native']['cycles']=19
    elif change=='memcheck-incomplete':data['reproduction']['memcheck']['cycles']=4
    elif change=='raw-row-failed':
        result=Path(data['reproduction']['native']['results_path']);record=json.loads(result.read_text())
        record['results'][0]['passed']=False;result.write_text(json.dumps(record))
        data['reproduction']['native']['results_sha256']=identity(result)['sha256']
    elif change=='raw-runtime-maps':
        maps=Path(data['reproduction']['native']['artifacts'][2]['path']);maps.write_text('unrelated mapping')
        data['reproduction']['native']['artifacts'][2]=identity(maps)
    elif change=='old-runtime':data['runtime_started_ns']=10**20
    if change not in ('missing-method','unknown-method','legacy-field','missing-review'):
        save_bound_record(data,'evidence',evidence)
        if change!='review-record-mismatch':review['evidence_sha256']=data['qualification']['evidence']['sha256']
        save_bound_record(data,'review',review)
    path.write_text(json.dumps(data))
    with pytest.raises(RuntimeError,match='BRIDGE_OVERLAY_REJECTED'):
        read_bridge_overlay(path,patch)


def test_installed_symlink_backend_finds_source_bound_admission_helper(tmp_path):
    path,patch,_,_,_=lifetime_admission_fixture(tmp_path)
    installed=tmp_path/'isolated-install/lib/workcell_builder';installed.mkdir(parents=True)
    (installed/'simulator_backend.py').symlink_to(SCRIPT.parent/'simulator_backend.py')
    code=('import sys;sys.path.insert(0,sys.argv[1]);from simulator_backend import read_bridge_overlay;'
          'read_bridge_overlay(sys.argv[2],sys.argv[3])')
    completed=subprocess.run([sys.executable,'-I','-c',code,str(installed),str(path),str(patch)],
                             text=True,capture_output=True)
    assert completed.returncode==0,completed.stderr


def test_publication_commits_only_complete_evidence(tmp_path):
    from bridge_shutdown_repro.qualification import publish
    from simulator_backend import read_bridge_overlay
    _,patch,data,_,_=lifetime_admission_fixture(tmp_path)
    publish(tmp_path,data,patch)
    assert read_bridge_overlay(tmp_path/'provenance.json',patch)['qualification']['method']=='deterministic-source-lifetime/v1'
    assert str(tmp_path) in (tmp_path/'setup.bash').read_text()
    assert not (tmp_path/'.provenance.json.tmp').exists()


def test_publication_failure_cannot_leave_discoverable_half_publication(tmp_path,monkeypatch):
    from bridge_shutdown_repro import qualification
    _,patch,data,_,_=lifetime_admission_fixture(tmp_path)
    original=qualification.os.replace
    def fail_manifest(source,destination):
        if Path(destination).name=='provenance.json':raise OSError('synthetic interrupted publication')
        return original(source,destination)
    monkeypatch.setattr(qualification.os,'replace',fail_manifest)
    with pytest.raises(ValueError,match='no publication'):
        qualification.publish(tmp_path,data,patch)
    assert not (tmp_path/'provenance.json').exists()
    assert not (tmp_path/'setup.bash').exists()


def test_runtime_rejects_boolean_returncodes(tmp_path):
    import json
    from bridge_shutdown_repro.qualification import identity,validate_runtime
    _,_,data,_,_=lifetime_admission_fixture(tmp_path)
    path=Path(data['reproduction']['native']['results_path']);record=json.loads(path.read_text())
    record['results'][0]['returncodes']=[False,False];path.write_text(json.dumps(record))
    data['reproduction']['native']['results_sha256']=identity(path)['sha256']
    with pytest.raises(ValueError,match='runtime cycle'):
        validate_runtime(data)


@pytest.mark.parametrize('bound_instrumentation',[True,False])
def test_instrumented_dependency_requires_its_own_source_and_build_binding(tmp_path,bound_instrumentation):
    import json
    from bridge_shutdown_repro.qualification import identity
    from simulator_backend import read_bridge_overlay
    path,patch,data,evidence,review=lifetime_admission_fixture(tmp_path)
    instrumented=tmp_path/'instrumented/librclcpp.so';instrumented.parent.mkdir()
    instrumented.write_text('SYNTHETIC instrumented pinned dependency; not runtime evidence')
    item=identity(instrumented);original=evidence['runtime']['dependencies'][0]
    for phase in ('before','after'):
        evidence[phase]['loaded_libraries'][0]=item
        maps=Path(evidence[phase]['maps']['path'])
        lines=maps.read_text().replace(str(Path(original['path']).stat().st_ino)+' '+original['path'],
                                      str(instrumented.stat().st_ino)+' '+str(instrumented))
        maps.write_text(lines);evidence[phase]['maps']=identity(maps)
        if bound_instrumentation:
            evidence[phase]['dependency_builds']=[dict(runtime_library=original,library=item,source_dependency='rclcpp',
                instrumentation=evidence['harness']['instrumentation'],build_config=evidence['harness']['build_config'])]
    save_bound_record(data,'evidence',evidence);review['evidence_sha256']=data['qualification']['evidence']['sha256']
    save_bound_record(data,'review',review);path.write_text(json.dumps(data))
    if bound_instrumentation:
        assert read_bridge_overlay(path,patch)['qualification']['method']=='deterministic-source-lifetime/v1'
    else:
        with pytest.raises(RuntimeError,match='dependency'):
            read_bridge_overlay(path,patch)


def test_finite_documented_harness_bounds_and_exit_codes_are_not_relabelled(tmp_path):
    import json
    from simulator_backend import read_bridge_overlay
    path,patch,data,evidence,review=lifetime_admission_fixture(tmp_path)
    evidence['finite_bound_seconds']=600
    evidence['before']['returncode']=97
    save_bound_record(data,'evidence',evidence);review['evidence_sha256']=data['qualification']['evidence']['sha256']
    save_bound_record(data,'review',review);path.write_text(json.dumps(data))
    assert read_bridge_overlay(path,patch)['qualification']['method']=='deterministic-source-lifetime/v1'


def test_admission_rejects_stale_builder_proof_identity(tmp_path):
    import json
    from simulator_backend import read_bridge_overlay
    path,patch,data,_,_=lifetime_admission_fixture(tmp_path)
    data['proof_script_sha256']='0'*64;path.write_text(json.dumps(data))
    with pytest.raises(RuntimeError,match='implementation hash'):
        read_bridge_overlay(path,patch)


def observed_admission_fixture(tmp_path):
    """Synthetic verifier inputs only; never observed shutdown evidence."""
    import json
    import os
    from bridge_shutdown_repro import qualification as q
    path,patch,manifest,old,review=lifetime_admission_fixture(tmp_path)
    def artifact(name,value):
        file=tmp_path/name;file.parent.mkdir(parents=True,exist_ok=True)
        file.write_text(value if isinstance(value,str) else json.dumps(value))
        return q.identity(file)
    def mapped(item):
        st=Path(item['path']).stat()
        return dict(item,device=st.st_dev,inode=st.st_ino)
    def mapping(items):
        return ''.join(f"1000-2000 r-xp 00000000 {os.major(Path(i['path']).stat().st_dev):02x}:{os.minor(Path(i['path']).stat().st_dev):02x} {Path(i['path']).stat().st_ino} {i['path']}\n" for i in items)
    baseline=artifact('baseline/parameter_bridge','SYNTHETIC installed baseline executable')
    manifest['baseline'].update(executable=baseline['path'],sha256=baseline['sha256'])
    evidence={k:old[k] for k in ('producer','source','patch','runtime')}
    evidence.update(schema='workcell_bridge_observed_shutdown/v1',method='observed-shutdown-order/v1',observations={})
    observer=[q.identity(SCRIPT.parent/'bridge_shutdown_repro'/name) for name in ('observe.py','observe.gdb.py')]
    publisher=artifact('observer/publisher','SYNTHETIC publisher')
    deferred='rclcpp::SignalHandler::deferred_signal_handler()'
    uninstall='rclcpp::SignalHandler::uninstall()'
    drop='spdlog::details::registry::drop(std::string const&)'
    destroy='spdlog::details::registry::~registry()'
    for phase,binary,base in (('baseline',baseline,3000),('patched',manifest['executable'],4000)):
        exp=dict(binary=mapped(binary),library=mapped(manifest['library']),publisher=mapped(publisher),
            config=q.identity(SCRIPT.parent/'bridge_shutdown_repro/bridges.yaml'),
            verifier=q.identity(SCRIPT.parent/'bridge_shutdown_repro/verify.py'),instrumentation=observer,
            cycles_limit=5,traffic_threshold=300,traffic_timeout_seconds=30,shutdown_timeout_seconds=30,
            environment=dict(manifest['runtime_environment']))
        rows=[];artifacts=[]
        for cycle in range(1,6):
            pid=base+cycle*10;worker=pid+3
            definitions=[('entry',deferred,worker,'signal'),('entry','rcl_logging_fini',worker,'logger')]
            if phase=='baseline':definitions += [('entry','exit',pid,'0')]
            definitions += [('entry',drop,worker,'registry'),('return',drop,worker,'registry')]
            if phase=='patched':definitions += [('return',deferred,worker,'signal'),('entry',uninstall,pid,'signal'),('return',uninstall,pid,'signal'),('entry','exit',pid,'0')]
            definitions += [('entry',destroy,pid,'registry'),('return',destroy,pid,'registry')]
            events=[]
            for seq,(kind,function,tid,obj) in enumerate(definitions,1):
                callers=('rcl_logging_external_shutdown\n#2 '+deferred if function.startswith('spdlog::details::registry::drop') else
                    '__run_exit_handlers\n#2 __GI_exit' if function==destroy else
                    'rclcpp::shutdown\n#2 spin_bridge_node\n#3 main' if function==uninstall else
                    deferred if function=='rcl_logging_fini' else 'start_thread' if function==deferred else '__libc_start_call_main')
                events.append(dict(seq=seq,at_ns=seq*1000000,kind=kind,pid=pid,tid=tid,function=function,object=obj,
                    stack='#0 '+function+'\n#1 '+callers))
            events.append(dict(seq=len(events)+1,at_ns=(len(events)+1)*1000000,kind='inferior_exited',pid=pid,tid=None,exit_code=0))
            events.append(dict(seq=len(events)+1,at_ns=(len(events)+1)*1000000,kind='observer_finished',pid=0,tid=None))
            values={'events.jsonl':''.join(json.dumps(e)+'\n' for e in events),
                'bridge.maps':mapping([binary,manifest['library'],*evidence['runtime']['dependencies']]),
                'debugger.log':f'[Inferior 1 (process {pid}) exited normally]\n',
                'publisher.log':'PUBLISHER_READY\n','inferior.pid':str(pid),
                'task_inventory.json':{str(pid):'parameter_bridg',str(worker):'parameter_bridg'}}
            artifacts.extend(artifact(f'observations/{phase}/{cycle:02}/{name}',value) for name,value in values.items())
            rows.append(dict(cycle=cycle,received=300,pids=dict(bridge=pid,publisher=pid+1,debugger=pid+2),
                groups=[pid,pid+1,pid+2],loaded_executable=binary['path'],
                loaded_identities=[mapped(item) for item in [manifest['library'],*evidence['runtime']['dependencies']]],
                signal_sent_monotonic_ns=1500000,publisher_returncode=0,debugger_returncode=0,bridge_returncode=0,
                observer_errors=[],debugger_terminated_inferior=False,access_after_destruction=False,
                remaining_processes=[],core_inventory_changed=False,clean_observation=True,
                commands=[['publisher',[publisher['path']]],['debugger',['/usr/bin/gdb','-q','-nx','-batch','-x',observer[1]['path'],'--args',binary['path'],'--ros-args','-p','config_file:='+exp['config']['path']]]]))
        evidence['observations'][phase]=dict(experiment=artifact(f'observations/{phase}/experiment.json',exp),
            results=artifact(f'observations/{phase}/results.json',rows),artifacts=artifacts)
    review['schema']='workcell_bridge_observed_shutdown_review/v1'
    review['assessment'].update(claim_limits='Synthetic fixture; ordering does not reproduce UAF.',uninstall_join='Synthetic source inspection of join.')
    review['implementation'].extend(observer)
    review['implementation'].extend(evidence['source']['files'])
    for item in evidence['source']['dependencies'].values():review['implementation'].extend(item['files'])
    for batch in evidence['observations'].values():review['implementation'].extend([batch['experiment'],batch['results'],*batch['artifacts']])
    review['implementation'].extend([evidence['runtime']['source_main'],publisher])
    review['implementation']=list({item['path']:item for item in review['implementation']}.values())
    manifest['qualification']['method']='observed-shutdown-order/v1'
    save_observed_fixture(manifest,evidence,review)
    path.write_text(json.dumps(manifest))
    return path,patch,manifest,evidence,review


def save_observed_fixture(manifest,evidence,review):
    from bridge_shutdown_repro.qualification import identity
    for batch in evidence['observations'].values():
        for key in ('experiment','results'):batch[key]=identity(batch[key]['path'])
        batch['artifacts']=[identity(item['path']) for item in batch['artifacts']]
    save_bound_record(manifest,'evidence',evidence)
    review['evidence_sha256']=manifest['qualification']['evidence']['sha256']
    review['implementation']=[identity(item['path']) for item in review['implementation']]
    save_bound_record(manifest,'review',review)


def test_observed_admission_requires_real_ordering_contract_without_abort_claim(tmp_path):
    from bridge_shutdown_repro import qualification as q
    _,_,data,evidence,_=observed_admission_fixture(tmp_path)
    assert q.method(data)=='observed-shutdown-order/v1'
    assert q.validate_observed(data)==evidence
    assert 'baseline_returncode' not in data['reproduction']


@pytest.mark.parametrize('observer_changed',[False,True])
def test_observed_admission_binds_reviewed_repository_bytes_not_the_checkout_location(tmp_path,observer_changed):
    """Evidence recorded from another checkout admits this one only for byte-identical reviewed files."""
    import json
    import shutil
    from bridge_shutdown_repro import qualification as q
    _,_,data,evidence,review=observed_admission_fixture(tmp_path)
    repo=SCRIPT.parents[1];old=tmp_path/'qualified-checkout'
    for name in q.REPOSITORY_FILES:
        (old/name).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(repo/name,old/name)
    if observer_changed:
        with (old/'scripts/bridge_shutdown_repro/observe.gdb.py').open('a') as script:script.write('# reviewed elsewhere\n')
    def relocate(value):return json.loads(json.dumps(value).replace(str(repo)+'/',str(old)+'/'))
    for batch in evidence['observations'].values():
        for key in ('experiment','results'):
            file=Path(batch[key]['path']);file.write_text(json.dumps(relocate(json.loads(file.read_text()))))
    data,evidence,review=relocate(data),relocate(evidence),relocate(review)
    save_observed_fixture(data,evidence,review)
    shutil.rmtree(old)       # the qualified checkout need not survive; its reviewed bytes must
    if observer_changed:
        with pytest.raises(ValueError,match='differs from the reviewed bytes: scripts/bridge_shutdown_repro/observe.gdb.py'):
            q.validate_observed(data)
    else:
        assert q.validate_observed(data)==evidence


@pytest.mark.parametrize('fault',['missing-cycle','traffic','nonzero-exit','forced-cleanup','forced-cleanup-flag','preload','canonical-dependency-omission','debugger-command','missing-environment','changed-rmw','runtime-interposition','survivor','core','wrong-library','missing-dependency','map-device','registry-identity','wrong-thread','missing-stack','early-exit','late-drop','missing-join','baseline-inactive','abort-claim','unbound-review','changed-source','runtime-before-review'])
def test_observed_admission_rejects_incomplete_or_misbound_shutdown_evidence(tmp_path,fault):
    import json
    from bridge_shutdown_repro import qualification as q
    _,_,data,evidence,review=observed_admission_fixture(tmp_path)
    phase='baseline' if fault=='baseline-inactive' else 'patched'
    batch=evidence['observations'][phase];results=Path(batch['results']['path'])
    rows=json.loads(results.read_text());row=rows[0]
    trace=results.parent/'01/events.jsonl';events=[json.loads(line) for line in trace.read_text().splitlines()]
    if fault=='missing-cycle':rows.pop()
    elif fault=='traffic':row['received']=299
    elif fault=='nonzero-exit':row['bridge_returncode']=-6
    elif fault=='forced-cleanup':row['debugger_terminated_inferior']=True
    elif fault=='forced-cleanup-flag':row['forced_cleanup']=True
    elif fault=='preload':
        exp_path=Path(batch['experiment']['path']);exp=json.loads(exp_path.read_text())
        exp['environment']={'LD_PRELOAD':'/unreviewed/instrumentation.so'};exp_path.write_text(json.dumps(exp))
    elif fault=='canonical-dependency-omission':
        omitted=evidence['runtime']['dependencies'].pop(0)
        data['qualification']['runtime_dependencies']=list(evidence['runtime']['dependencies'])
        for other in evidence['observations'].values():
            other_path=Path(other['results']['path']);other_rows=json.loads(other_path.read_text())
            for record in other_rows:record['loaded_identities']=[i for i in record['loaded_identities'] if i['path']!=omitted['path']]
            other_path.write_text(json.dumps(other_rows))
        for record in rows:record['loaded_identities']=[i for i in record['loaded_identities'] if i['path']!=omitted['path']]
    elif fault=='debugger-command':row['commands'][1][1][1:1]=['-ex','set scheduler-locking on']
    elif fault=='missing-environment':
        exp_path=Path(batch['experiment']['path']);exp=json.loads(exp_path.read_text());exp.pop('environment');exp_path.write_text(json.dumps(exp))
    elif fault=='changed-rmw':data['runtime_environment']['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
    elif fault=='runtime-interposition':data['runtime_environment']['VALGRIND_OPTS']='--suppressions=/tmp/hide'
    elif fault=='survivor':row['remaining_processes']=[row['pids']['bridge']]
    elif fault=='core':row['core_inventory_changed']=True
    elif fault=='wrong-library':row['loaded_identities'][0]['sha256']='0'*64
    elif fault=='missing-dependency':row['loaded_identities'].pop()
    elif fault=='map-device':
        maps=results.parent/'01/bridge.maps';maps.write_text(maps.read_text().replace(' r-xp 00000000 ',' r-xp 00000000 ff:ff ',1))
    elif fault=='registry-identity':next(e for e in events if e.get('function','').endswith('~registry()'))['object']='another-object'
    elif fault=='wrong-thread':next(e for e in events if e.get('function','').endswith('uninstall()'))['tid']=row['pids']['bridge']+3
    elif fault=='missing-stack':next(e for e in events if e.get('function','').startswith('spdlog::details::registry::drop'))['stack']=''
    elif fault=='early-exit':
        i=next(i for i,e in enumerate(events) if e.get('function')=='exit');events.insert(2,events.pop(i))
    elif fault=='late-drop':
        i=next(i for i,e in enumerate(events) if e.get('function','').startswith('spdlog::details::registry::drop') and e['kind']=='return');events.insert(-2,events.pop(i))
    elif fault=='missing-join':events=[e for e in events if not e.get('function','').endswith('uninstall()')]
    elif fault=='baseline-inactive':
        i=next(i for i,e in enumerate(events) if e.get('function')=='exit');events.insert(5,events.pop(i))
    elif fault=='abort-claim':data['reproduction']['baseline_returncode']=-6
    elif fault=='unbound-review':review['implementation']=[i for i in review['implementation'] if not i['path'].endswith('observe.gdb.py')]
    elif fault=='changed-source':evidence['source']['bridge_commit']='0'*40
    elif fault=='runtime-before-review':data['runtime_started_ns']=review['reviewed_at_ns']-1
    for seq,event in enumerate(events,1):event.update(seq=seq,at_ns=seq*1000000)
    trace.write_text(''.join(json.dumps(e)+'\n' for e in events));results.write_text(json.dumps(rows))
    save_observed_fixture(data,evidence,review)
    with pytest.raises(ValueError):q.validate_observed(data)


def test_observed_builder_requires_both_bound_records_and_memcheck():
    parser=MODULE.argument_parser()
    args=['--output','o','--qualification-method','observed-shutdown-order/v1']
    for extra in ([],['--lifetime-evidence','record.json'],
                  ['--mechanism-review','review.json']):
        with pytest.raises(SystemExit):
            MODULE.validate_arguments(parser,parser.parse_args(args+extra))
    complete=args+['--lifetime-evidence','record.json','--mechanism-review','review.json',
                   '--valgrind','/usr/bin/valgrind']
    parsed=parser.parse_args(complete)
    MODULE.validate_arguments(parser,parsed)
    assert parsed.qualification_method=='observed-shutdown-order/v1'
    with pytest.raises(SystemExit):
        MODULE.validate_arguments(parser,parser.parse_args(complete[:-1]+['/bin/true']))


def observed_fatal_fixture(tmp_path):
    """Synthetic archived crash fields test validation, never reproduce a crash."""
    import json
    from bridge_shutdown_repro import qualification as q
    path,patch,data,evidence,review=observed_admission_fixture(tmp_path)
    batch=evidence['observations']['baseline'];root=Path(batch['results']['path']).parent
    rows=json.loads((root/'results.json').read_text())[:1];row=rows[0];pid=row['pids']['bridge'];worker=pid+3
    events=[json.loads(line) for line in (root/'01/events.jsonl').read_text().splitlines()][:3]
    stack=('#0 0x1234 in eprosima::fastrtps::rtps::RTPSWriter::new_change()\n'
           '#1 rmw_fastrtps_shared_cpp::__rmw_publish()\n#2 rmw_fastrtps_shared_cpp::__rmw_destroy_publisher()\n'
           '#3 rcl_logging_fini\n#4 rclcpp::Context::shutdown()\n#5 rclcpp::SignalHandler::deferred_signal_handler()\n')
    events.extend([dict(seq=4,at_ns=4000000,kind='signal',pid=pid,tid=worker,signal='SIGSEGV',stack=stack),
                   dict(seq=5,at_ns=5000000,kind='inferior_exited',pid=pid,tid=None,exit_code=None)])
    (root/'01/events.jsonl').write_text(''.join(json.dumps(event)+'\n' for event in events))
    (root/'01/debugger.log').write_text("Thread 3 received signal SIGSEGV\ngdb.error: Couldn't get registers: No such process.\n")
    row.update(bridge_returncode=None,clean_observation=False,core_inventory_changed=True)
    (root/'results.json').write_text(json.dumps(rows))
    batch['artifacts']=[item for item in batch['artifacts'] if '/01/' in item['path']]
    review['implementation']=[item for item in review['implementation'] if not (str(root) in item['path'] and any('/'+f'{i:02}'+'/' in item['path'] for i in range(2,6)))]
    terminal=root/'terminal';terminal.mkdir()
    values=dict(signal='11\n',executable_path=data['baseline']['executable']+'\n',
        proc_status=f'Pid:\t{pid}\nTgid:\t{pid}\nPPid:\t{row["pids"]["debugger"]}\nTracerPid:\t{row["pids"]["debugger"]}\n',
        proc_maps=(root/'01/bridge.maps').read_text(),
        backtrace=f'Program terminated with signal SIGSEGV\nThread 1 (Thread 0x123 (LWP {worker})):\n'+stack+
          f'\nThread 2 (Thread 0x456 (LWP {pid})):\n#0 eprosima::fastdds::dds::DomainParticipantFactory::~DomainParticipantFactory()\n#1 __run_exit_handlers\n',
        registers='Program terminated with signal SIGSEGV\nrip 0x1234 RTPSWriter::new_change\nrdi 0x0\n')
    batch['terminal']={}
    for name,value in values.items():
        file=terminal/name;file.write_text(value);batch['terminal'][name]=q.identity(file)
    core=terminal/'core';core.write_bytes(b'\x7fELF\x02\x01'+b'\0'*10+b'\x04\x00'+b'\0'*46)
    batch['terminal']['core']=q.identity(core)
    review['implementation'].extend(batch['terminal'].values())
    save_observed_fixture(data,evidence,review);path.write_text(json.dumps(data))
    return data,evidence,review


def test_observed_baseline_accepts_attributable_natural_failure_without_relabelling_it(tmp_path):
    from bridge_shutdown_repro.qualification import validate_observed
    data,evidence,_=observed_fatal_fixture(tmp_path)
    assert validate_observed(data)==evidence
    assert 'baseline_returncode' not in data['reproduction']


@pytest.mark.parametrize('fault',['wrong-core-pid','wrong-core-signal','unrelated-fault','fault-before-exit','missing-main-stack','missing-worker-stack','bad-core','wrong-registers','core-not-reviewed','patched-fatal','fake-returncode','forced-cleanup','observer-kill'])
def test_observed_fatal_baseline_rejects_unattributed_or_observer_caused_failures(tmp_path,fault):
    import json
    from bridge_shutdown_repro import qualification as q
    data,evidence,review=observed_fatal_fixture(tmp_path)
    batch=evidence['observations']['baseline'];root=Path(batch['results']['path']).parent
    terminal=batch['terminal'];rows=json.loads((root/'results.json').read_text());row=rows[0]
    trace=root/'01/events.jsonl';events=[json.loads(line) for line in trace.read_text().splitlines()]
    if fault=='wrong-core-pid':Path(terminal['proc_status']['path']).write_text('Pid: 999\nTgid: 999\nPPid: 999\nTracerPid: 999\n')
    elif fault=='wrong-core-signal':Path(terminal['signal']['path']).write_text('6\n')
    elif fault=='unrelated-fault':events[3]['stack']='#0 unrelated_function\n#1 main'
    elif fault=='fault-before-exit':events[2],events[3]=events[3],events[2]
    elif fault=='missing-main-stack':
        p=Path(terminal['backtrace']['path']);p.write_text(p.read_text().replace('__run_exit_handlers','unrelated'))
    elif fault=='missing-worker-stack':
        p=Path(terminal['backtrace']['path']);p.write_text(p.read_text().replace('rcl_logging_fini','unrelated'))
    elif fault=='bad-core':Path(terminal['core']['path']).write_bytes(b'not an ELF core')
    elif fault=='wrong-registers':Path(terminal['registers']['path']).write_text('rip 0x7777\n')
    elif fault=='core-not-reviewed':review['implementation']=[i for i in review['implementation'] if i['path']!=terminal['core']['path']]
    elif fault=='patched-fatal':evidence['observations']['patched']['terminal']=terminal
    elif fault=='fake-returncode':row['bridge_returncode']=-11
    elif fault=='forced-cleanup':row['forced_cleanup']=True
    elif fault=='observer-kill':row['debugger_terminated_inferior']=True
    for seq,event in enumerate(events,1):event.update(seq=seq,at_ns=seq*1000000)
    trace.write_text(''.join(json.dumps(e)+'\n' for e in events));(root/'results.json').write_text(json.dumps(rows))
    batch['terminal']={key:q.identity(item['path']) for key,item in terminal.items()}
    save_observed_fixture(data,evidence,review)
    with pytest.raises(ValueError):q.validate_observed(data)


@pytest.mark.parametrize('variable',['LD_PRELOAD','LD_AUDIT','VALGRIND_LIB','VALGRIND_OPTS'])
@pytest.mark.parametrize('method',['observed-shutdown-order/v1','deterministic-source-lifetime/v1'])
def test_alternative_builder_rejects_ambient_interposition(tmp_path,monkeypatch,variable,method):
    for key in ('LD_PRELOAD','LD_AUDIT','VALGRIND_LIB','VALGRIND_OPTS'):monkeypatch.delenv(key,raising=False)
    monkeypatch.setenv(variable,'unreviewed-instrumentation')
    parser=MODULE.argument_parser()
    args=parser.parse_args(['--output','o','--qualification-method',method,'--lifetime-evidence','record.json',
                           '--mechanism-review','review.json','--valgrind','/usr/bin/valgrind'])
    with pytest.raises(SystemExit):MODULE.validate_arguments(parser,args)
