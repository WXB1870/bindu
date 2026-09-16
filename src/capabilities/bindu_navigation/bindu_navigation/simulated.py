class SimNavigation:
    async def navigate(self, port, context, site):
        # Test fixture; no map localization or Nav2 arrival guarantee.
        velocity, duration = {'pickup': (.2, .3), 'home': (-.2, .3)}[site]
        await port.base_segment(context, velocity, duration)
