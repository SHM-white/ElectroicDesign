# Open Source Use Boundary

This is an engineering compliance record, not legal advice. A qualified reviewer must assess the actual distribution, deployment, modifications, recipients, and jurisdiction before a release or event handoff.

## Current Scope

Internal academic-competition development is the approved working scope. That scope does not remove license duties if the project later distributes source, binaries, containers, images, model artifacts, or access to a covered network service. The exact upstream revisions, license snapshots, notices, and source locations are recorded in the provenance manifests and checked by `python3 tools/check_third_party.py --strict`.

## Invocation Boundaries

Livox ROS Driver 2 and FAST-LIO remain independently imported and launched from `ros2_ws/src/third_party`. Project-owned `ed_*` packages communicate only through declared ROS interfaces and must not copy their sources. Future Ultralytics training/export runs remain in the isolated `ml/yolo` environment; ROS runtime code consumes provider-neutral outputs and must not import Ultralytics.

These separate-process boundaries preserve technical ownership and traceability. They do not, by themselves, determine whether a combined artifact is a derivative or combined work under any license; that question requires fact-specific legal review.

## Copyleft Review Points

FAST-LIO is recorded as GPL-2.0-only. Distribution of the covered program or a modified/combined artifact can create notice, license-text, and corresponding-source obligations. Ultralytics is recorded as AGPL-3.0-only. In addition to distribution questions, AGPL can require an offer of corresponding source to users who interact with a modified covered program over a network. The project therefore keeps its use isolated, pins the upstream source, and blocks model-weight downloads until task-specific provenance exists.

When an obligation is triggered, preserve applicable notices and provide the corresponding source for the covered version and modifications through the required channel and timing. Publishing material only after an event can support compliance work, but it does not retroactively bypass an obligation that already applied. MIT-licensed Livox material still requires its applicable copyright and permission notice to be retained.

## Release Gate

Before any redistribution, competition delivery, public service, or image/container handoff, review the exact artifact composition, the relevant GPL/AGPL conditions, all modifications, source-offer mechanics, model and dataset licenses, and recipient access. Do not represent this document as a legal conclusion.
