import copy
import tempfile
from pathlib import Path
import unittest
from r2handoversim.paper_replay import generate, read_records, verify, write_records, materialize
from r2handoversim.evaluation import validate_trial


class PaperReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records = read_records()

    def test_bundled_records_match_every_paper_cell(self):
        audit = verify(self.records)
        self.assertEqual(audit['records'], 8000)
        self.assertEqual(audit['checked_table_cells'], 108)
        self.assertFalse(audit['original_experiments_executed'])

    def test_reproducible_and_rejects_missing_or_modified_records(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp)/'a.gz', Path(tmp)/'b.gz'
            write_records(a, generate()); write_records(b, self.records)
            self.assertEqual(a.read_bytes(), b.read_bytes())
        with self.assertRaisesRegex(ValueError, 'unbalanced'):
            verify(self.records[:-1])
        changed = copy.deepcopy(self.records)
        changed[0]['stage_time_s']['total'] += 1
        with self.assertRaisesRegex(ValueError, 'timing'):
            verify(changed)

    def test_all_outcomes_materialize_without_claiming_measurements(self):
        seen = set()
        for r in self.records:
            if r['outcome'] in seen: continue
            seen.add(r['outcome'])
            trial = materialize(r, samples=3)
            validate_trial(trial)
            self.assertEqual(trial['replay_reference']['assigned_outcome'], r['outcome'])
            self.assertNotIn('success', trial)
        self.assertEqual(len(seen), 6)
