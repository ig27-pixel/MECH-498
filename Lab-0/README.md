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

From this folder:

```bash
python investment_student.py
```

The script prints the retirement scenario results and may open matplotlib plots
depending on the code path used.

## Notes

- The main work is in `StudentInvestment`.
- Dollar amounts use the default salary and contribution rules from
  `investment_base.py`.
- `*.Zone.Identifier` files are Windows download metadata and are not part of
  the lab logic.
