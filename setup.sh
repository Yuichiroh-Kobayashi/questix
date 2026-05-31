#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
ansible-playbook ansible/playbooks/setup_kit.yaml -i localhost, --connection=local --ask-become-pass
