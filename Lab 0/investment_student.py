#!/usr/bin/env python3
from investment_base import InvestmentBase
from investment_fund import InvestmentFund #HINT -- you will need to add something here


class StudentInvestment(InvestmentBase):
  """
  Student implementation of retirement investment calculations.

  Implements all abstract methods from InvestmentBase to provide
  concrete calculations for 401k planning, compound interest,
  and various retirement scenarios.
  """

  def __init__(self):
    """Initialize StudentInvestment with parent class defaults."""
    super().__init__()

  def calculate_investment_compounded_annually(self, principal: float, contribution: float, fund: InvestmentFund, start_year: float, end_year: float):
    """
    Calculate compound interest with annual contributions.

    Args:
      principal (float): Initial investment amount
      contribution (float): Annual contribution amount
      fund (InvestmentFund): InvestmentFund object with yearly return rates
      start_year (float): Starting year for calculations
      end_year (float): Ending year for calculations (inclusive)

    Returns:
      float: Total investment value after compound growth
    """
    ## TODO -- implement this method
    pass 

  def calculate_401k_match(self, input_percent: float) -> float:
    """
    Calculate employer 401k matching based on contribution percentage.

    Employer matches:
    - 100% of first 2% contributed
    - 50% of next 6% contributed

    Args:
      input_percent (float): Employee contribution percentage

    Returns:
      float: Employer matching amount in dollars
    """
    ## TODO -- implement this method
    pass
    
  def calculate_total_contribution(self, your_contribution: float, employer_contribution: float) -> float:
    """
    Calculate total annual contribution with IRS limit of $24,500.

    Args:
      your_contribution (float): Individual contribution amount
      employer_contribution (float): Employer matching amount

    Returns:
      float: Total contribution amount (capped at $24,500 IRS limit)
    """
    ## TODO -- implement this method
    return min(...)

  def calculate_conservative_retirement(self):
    """
    Calculate retirement value using conservative AFund strategy.

    Contributes 10% of salary with employer match using AFund
    from 2025 to 2065.

    Returns:
      float: Total retirement value using conservative fund
    """
    ## TODO -- implement this method
    pass
  
  def calculate_first_10(self):
    """
    Calculate retirement value contributing for first 10 years only.

    Contributes 6% of salary for 10 years (2025-2035) then stops,
    letting investment grow with no additional contributions.

    Returns:
      float: Total retirement value after early contributions strategy
    """
    ## TODO -- implement this method
    pass
  
  def calculate_last_30(self):
    """
    Calculate retirement value contributing for last 30 years only.

    Waits until 2035 then contributes for remaining
    30 years until retirement.

    Returns:
      float: Total retirement value using late contributions strategy
    """
    ## TODO -- implement this method
    pass
  
  def calculate_risky_retirement_2065(self):
    """
    Calculate retirement value using risky CFund through 2065.

    Contributes 10% of salary with employer match using volatile
    CFund from 2025 to 2065.

    Returns:
      float: Total retirement value using risky fund (before market crash)
    """
    ## TODO -- implement this method
    pass
  
  def calculate_risky_retirement_2066(self):
    """
    Calculate retirement value using risky CFund through 2066.

    Contributes 10% of salary with employer match using volatile
    CFund from 2025 to 2066 (including the -36.1% crash year).

    Returns:
      float: Total retirement value using risky fund (after market crash)
    """
    ## TODO -- implement this method
   

  

if __name__ == "__main__":
  investment = StudentInvestment()
  investment.plot_growth_over_time(
    principal=0,
    your_contribution=investment.salary * 0.10,
    employer_contribution=investment.calculate_401k_match(10),
    fund=AFund(),
    start_year=2035,
    end_year=2045
  )