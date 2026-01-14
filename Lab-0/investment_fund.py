
"""
NO MODIFICATION BY STUDENT IS REQUIRED

Investment fund classes providing historical return rates for retirement planning.

Contains fund classes with yearly return data for different investment strategies:
conservative (AFund), moderate volatility (BFund), and high volatility (CFund).
"""


class InvestmentFund:
  """
  Abstract base class for investment funds with yearly return rates.

  Provides interface for retrieving annual return rates by year.
  Subclasses must populate _return_rates dictionary with year: percentage mappings.
  """

  def __init__(self):
    """Initialize empty return rates dictionary."""
    self._return_rates: dict = {}

  def get_rate_by_year(self, year: float) -> float:
    """
    Get annual return rate for specified year as decimal.

    Args:
      year (float): Year to get return rate for

    Returns:
      float: Annual return rate as decimal (e.g., 0.05 for 5%)

    Raises:
      NotImplementedError: If subclass hasn't implemented return rates
      ValueError: If year is not in the return rates data
    """
    if self._return_rates == {}:
      raise NotImplementedError("Subclasses must implement return rates.")
    if year not in self._return_rates:
      raise ValueError("Start year or end year not in rates data.")
    return self._return_rates[year] / 100.0
    
    
  
class AFund(InvestmentFund):
  """
  Conservative investment fund with stable, lower-volatility returns.

  Represents a conservative investment strategy similar to government bonds
  or stable value funds. Returns range from approximately 1-9% annually
  with minimal negative returns.
  """

  def __init__(self):
    """
    Initialize AFund with conservative return rates from 2025-2065.
    """
    super().__init__()
    self._return_rates = {
      2025: 8.81,
      2026: 8.81,
      2027: 8.90,
      2028: 8.15,
      2029: 7.23,
      2030: 6.14,
      2031: 7.22,
      2032: 7.03,
      2033: 6.76,
      2034: 6.77,
      2035: 5.74,
      2036: 5.99,
      2037: 6.42,
      2038: 5.39,
      2039: 5.00,
      2040: 4.11,
      2041: 4.30,
      2042: 4.49,
      2043: 4.93,
      2044: 4.87,
      2045: 3.75,
      2046: 2.97,
      2047: 2.81,
      2048: 2.45,
      2049: 1.47,
      2050: 1.89,
      2051: 2.31,
      2052: 2.04,
      2053: 1.82,
      2054: 2.33,
      2055: 2.91,
      2056: 2.24,
      2057: 0.97,
      2058: 1.38,
      2059: 2.98,
      2060: 4.22,
      2061: 4.40,
      2062: 3.25,
      2063: 4.01,
      2064: 1.38,
      2065: 1.58,
    }

class BFund(InvestmentFund):
  """
  Moderate volatility investment fund with mixed returns.

  Represents a balanced investment strategy with moderate risk.
  Returns show higher volatility than AFund, ranging from
  approximately -26% to +37% annually.
  """

  def __init__(self):
    """
    Initialize BFund with moderate volatility return rates from 2025-2065.
    """
    super().__init__()
    self._return_rates = {
    2025: -9.04,
    2026: -18.14,
    2027: 36.92,
    2028: 18.03,
    2029: 10.45,
    2030: 15.30,
    2031: 5.49,
    2032: -20.32,
    2033: 34.85,
    2034: 20.06,
    2035: -3.38,
    2036: 18.57,
    2037: 30.35,
    2038: 7.80,
    2039: -2.92,
    2040: 16.35,
    2041: 18.22,
    2042: -9.26,
    2043: 27.97,
    2044: 25.85,
    2045: 12.45,
    2046: -26.26,
    2047: 25.30,
    2048: 16.93,
    2049: 5.17,
    2050: 18.91,
    2051: 7.59,
    2052: 10.12,
    2053: -14.66,
    2054: 19.34,
    2055: 24.75,
    2056: 6.17,
    2057: 12.28,
    2058: -14.87,
    2059: 19.34,
    2060: 16.89,
    2061: 7.25,
    2062: 12.01,
    2063: -10.75,
    2064: 14.12,
    2065: 8.56,
    }
    

class CFund(InvestmentFund):
  """
  High volatility investment fund with aggressive growth potential.

  Represents a risky investment strategy similar to growth stocks
  or emerging markets. Returns show high volatility ranging from
  approximately -36% to +37% annually.
  """

  def __init__(self):
    """
    Initialize CFund with high volatility return rates from 2025-2066.
    """
    super().__init__()
    self._return_rates = {
    2025: 12.40,
    2026: 27.25,
    2027: -6.56,
    2028: 26.31,
    2029: 4.46,
    2030: 7.06,
    2031: 1.32,
    2032: 37.58,
    2033: 22.96,
    2034: 33.36,
    2035: 28.58,
    2036: 21.04,
    2037: -9.10,
    2038: -11.89,
    2039: -22.10,
    2040: 28.67,
    2041: 10.88,
    2042: 4.91,
    2043: 15.79,
    2044: 5.49,
    2045: -36.78,
    2046: 26.46,
    2047: 15.10,
    2048: 2.11,
    2049: 15.90,
    2050: 32.15,
    2051: 13.69,
    2052: 1.38,
    2053: 12.01,
    2054: 21.82,
    2055: -4.41,
    2056: 28.88,
    2057: 18.40,
    2058: 28.68,
    2059: -19.44,
    2060: 26.50,
    2061: 17.50,
    2062: 10.38,
    2063: 13.91,
    2064: 5.21,
    2065: 14.32,
    2066: -36.1
    }
    