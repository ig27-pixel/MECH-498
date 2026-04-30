# Lab 0

Introductory Python warm-up for the MECH 498 coding workflow. This lab uses a
retirement-investment example to practice classes, inheritance, method
implementation, and plotting.

## Files

- `investment_base.py`: provided abstract base class and plotting helper.
- `investment_fund.py`: fund return-rate models used by the calculations.
- `investment_student.py`: student implementation of compound growth, 401k
  matching, contribution caps, and retirement scenarios.
- `498-2026-lab0.pdf`: original assignment handout.

## Run

From this folder or from the repository root:

```bash
# from Lab-0/
python investment_student.py

# from repository root
python Lab-0/investment_student.py
```

The script prints the retirement scenario results and opens matplotlib plots
for each scenario.

## Notes

- All student logic lives in the `StudentInvestment` class in
  `investment_student.py`.
- Dollar amounts and salary defaults come from `investment_base.py`; do not
  modify the base class.
