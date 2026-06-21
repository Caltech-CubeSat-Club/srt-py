"""navbar.py

Contains Functions for Interacting with a Navbar

"""

import dash_bootstrap_components as dbc


def generate_navbar(dropdowns, title="Commands", extra_buttons=None):
    """Generates the Navbar

    Parameters
    ----------
    dropdowns : dict
        Dictionary of Buttons for Each Dropdown Menu
    extra_buttons : list, optional
        List of additional button-like components to render in toolbar.
    title : str
        Title of the Navbar

    Returns
    -------
    NavbarSimple
    """
    extra_buttons = extra_buttons or []
    navbar = dbc.NavbarSimple(
        [*extra_buttons]
        + [
            dbc.DropdownMenu(
                children=dropdowns[drop_down],
                in_navbar=True,
                label=drop_down,
                style={"display": "flex", "flexWrap": "wrap"},
                className="m-1",
            )
            for drop_down in dropdowns
        ],
        brand=title,
        brand_style={"font-size": "large"},
        color="secondary",
        dark=True,
    )
    return navbar
