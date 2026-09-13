from accounts.views import EMAIL_VERIFICATION_LINK


def global_site_settings(request):
    if not request.user.is_authenticated or not request.user.is_viewer:
        return {}
    return {
        "SHOULD_SHOW_EMAIL_VERIFICATION": True,
        "EMAIL_VERIFICATION_LINK": EMAIL_VERIFICATION_LINK,
    }
