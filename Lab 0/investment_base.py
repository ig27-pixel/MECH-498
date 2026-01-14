#!/usr/bin/env python3
"""
NO MODIFIATION BY STUDENT IS REQUIRED

Investment base class providing abstract methods for retirement planning calculations.
"""
import matplotlib.pyplot as plt
from investment_fund import InvestmentFund

class InvestmentBase(object):
  """
  Abstract base class for investment calculations and retirement planning.

  Provides common functionality and defines interface for investment calculations
  including 401k matching, compound interest, and retirement scenario planning.
  """

  @property
  def salary(self):
    """Get the annual salary in dollars."""
    return self._salary

  def __init__(self):
    """Initialize investment base with default salary and principal."""
    self._salary = 100000
    self._current_principal = 0

  def plot_growth_over_time(self,
                            principal: float,
                            your_contribution: float,
                            employer_contribution: float,
                            fund: InvestmentFund,
                            start_year: float,
                            end_year: float):
    """
    Plot investment growth over time showing contributions and fund value.

    Args:
      principal (float): Initial investment amount
      your_contribution (float): Annual individual contribution
      employer_contribution (float): Annual employer contribution
      fund (InvestmentFund): InvestmentFund object with return rates
      start_year (float): Starting year for calculations
      end_year (float): Ending year for calculations
    """
    years = []
    cumulative_individual = []
    cumulative_employer = []
    cumulative_total_contributions = []
    cumulative_fund_values = []

    total_contribution = your_contribution + employer_contribution

    for year in range(int(start_year)+1, int(end_year) + 1):
      years.append(year)

      years_elapsed = year - start_year

      cum_individual = your_contribution * years_elapsed
      cum_employer = employer_contribution * years_elapsed
      cum_total = principal + total_contribution * years_elapsed
      cum_fund_value = self.calculate_investment_compounded_annually(principal, total_contribution, fund, start_year, year)

      cumulative_individual.append(cum_individual)
      cumulative_employer.append(cum_employer)
      cumulative_total_contributions.append(cum_total)
      cumulative_fund_values.append(cum_fund_value)

    plt.figure(figsize=(12, 8))
    plt.plot(years, cumulative_individual, label='Cumulative Individual Contribution', linewidth=2)
    plt.plot(years, cumulative_employer, label='Cumulative Employer Contribution', linewidth=2)
    plt.plot(years, cumulative_total_contributions, label='Cumulative Total Contributions', linewidth=2)
    plt.plot(years, cumulative_fund_values, label='Cumulative Fund Value', linewidth=2)

    plt.xlabel('Year')
    plt.ylabel('Value ($k)')
    plt.title('Investment Growth Over Time')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()

  def calculate_investment_compounded_annually(self,
                                             principal: float,
                                             contribution: float,
                                             fund: InvestmentFund,
                                             start_year: float,
                                             end_year: float):
    """
    Calculate compound interest with annual contributions.

    Args:
      principal (float): Initial investment amount
      contribution (float): Annual contribution amount
      fund (InvestmentFund): InvestmentFund object with return rates
      start_year (float): Starting year for calculations
      end_year (float): Ending year for calculations

    Returns:
      float: Total investment value after compound growth
    """
    raise NotImplementedError("Student must implement this method in their class.")  
  
  def calculate_401k_match(self, input_percent: float) -> float:
    """
    Calculate employer 401k matching contribution.

    Args:
      input_percent (float): Employee contribution percentage

    Returns:
      float: Employer matching amount in dollars
    """
    raise NotImplementedError("Student must implement this method in their class.")

  def calculate_total_contribution(self, your_contribution: float, employer_contribution: float) -> float:
    """
    Calculate total annual contribution with IRS limits of $24,500.

    Args:
      your_contribution (float): Individual contribution amount
      employer_contribution (float): Employer matching amount

    Returns:
      float: Total contribution amount (capped at IRS limits)
    """
    raise NotImplementedError("Student must implement this method in their class.")

  def calculate_conservative_retirement(self):
    """
    Calculate retirement value using conservative AFund strategy.

    Contributes 10% of salary with employer match using AFund
    from 2025 to 2065.

    Returns:
      float: Total retirement value using conservative fund
    """
    raise NotImplementedError("Student must implement this method in their class.")

  def calculate_first_10(self):
    """
    Calculate retirement value contributing for first 10 years only.

    Contributes 6% of salary for 10 years (2025-2035) then stops,
    letting investment grow with no additional contributions.

    Returns:
      float: Total retirement value after early contributions strategy
    """
    raise NotImplementedError("Student must implement this method in their class.")

  def calculate_last_30(self):
    """
    Calculate retirement value contributing for last 30 years only.

    Waits until 2035 then contributes for remaining
    30 years until retirement.

    Returns:
      float: Total retirement value using late contributions strategy
    """
    raise NotImplementedError("Student must implement this method in their class.")

  def calculate_risky_retirement_2065(self):
    """
    Calculate retirement value using risky CFund through 2065.

    Contributes 10% of salary with employer match using volatile
    CFund from 2025 to 2065.

    Returns:
      float: Total retirement value using risky fund (before market crash)
    """
    raise NotImplementedError("Student must implement this method in their class.")

  def calculate_risky_retirement_2066(self):
    """
    Calculate retirement value using risky CFund through 2066.

    Contributes 10% of salary with employer match using volatile
    CFund from 2025 to 2066 (including the -36.1% crash year).

    Returns:
      float: Total retirement value using risky fund (after market crash)
    """
    raise NotImplementedError("Student must implement this method in their class.")

