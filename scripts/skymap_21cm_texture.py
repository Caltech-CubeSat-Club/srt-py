import numpy as np

# Set up matplotlib
import matplotlib.pyplot as plt

from astropy.io import fits
from astropy.table import Table

hdu_list = fits.open("scripts/STOCKERT+VILLA-ELISA_1420MHz_1_256.fits")
