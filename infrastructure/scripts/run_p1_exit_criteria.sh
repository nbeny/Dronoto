#!/usr/bin/env bash
# Critere de sortie de P1 : les scenarios passent N fois de suite.
#
# Une phase n'est pas terminee parce que le code est ecrit, mais parce que la
# metrique est atteinte (docs/architecture/14-tests.md, section 10).
#
# Duree : 45 a 90 minutes pour N=10.
# Pas de « set -e » : on veut compter les echecs, pas s'arreter au premier.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

RUNS="${RUNS:-10}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RESULTS="tests/results/p1_exit_$STAMP"
mkdir -p "$RESULTS"

pass=0
fail=0
failed_runs=()

for i in $(seq 1 "$RUNS"); do
  echo "===== execution $i/$RUNS ====="
  if pytest -m scenario -v --junitxml="$RESULTS/run_$i.xml" \
       > "$RESULTS/run_$i.log" 2>&1; then
    pass=$((pass + 1))
    echo "  REUSSITE"
  else
    fail=$((fail + 1))
    failed_runs+=("$i")
    echo "  ECHEC (voir $RESULTS/run_$i.log)"
    tail -20 "$RESULTS/run_$i.log" | sed 's/^/    /'
  fi
  # Laisser les processus se terminer avant l'execution suivante : un orphelin
  # rendrait le resultat suivant non reproductible.
  sleep 5
done

echo
echo "===== resultat ====="
echo "  reussites : $pass / $RUNS"
echo "  echecs    : $fail / $RUNS"
[ ${#failed_runs[@]} -gt 0 ] && echo "  executions en echec : ${failed_runs[*]}"
echo "  rapports  : $RESULTS"
echo

if [ "$fail" -eq 0 ]; then
  echo "  CRITERE DE SORTIE P1 : ATTEINT"
  exit 0
fi

cat <<'DIAG'
  CRITERE DE SORTIE P1 : NON ATTEINT

  Un test qui passe neuf fois sur dix revele une condition de course, un delai
  trop serre ou un processus orphelin. Le diagnostiquer maintenant coute une
  journee ; le laisser passer coute des semaines de tests intermittents en P4.

  Pistes selon le mode d'echec :
    echec toujours sur la meme phase   -> bug deterministe, rejouer en
                                          headless: false et observer
    echec aleatoire au demarrage       -> px4_boot_wait_s trop court
                                          (tests/runner/sim_process.py)
    echec apres plusieurs executions   -> orphelins : ps aux | grep -E 'px4|gz sim'
    distance_to_home juste au-dessus   -> reglage du suivi de trajectoire,
                                          pas un defaut logiciel
DIAG
exit 1
