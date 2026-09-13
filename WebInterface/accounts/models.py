from django.conf import settings
from django.contrib.auth.models import AbstractUser
from django.db import models

ROLES = {
    "view_only": "View Only",
    "student": "Student",
    "instructor": "Instructor",
}
ROLE_COLORS = {
    "view_only": "danger",
    "student": "primary",
    "instructor": "warning"
}


class CustomUser(AbstractUser):
    is_viewer = models.BooleanField(default=False)
    is_instructor = models.BooleanField(default=False)
    role = models.CharField(max_length=20, choices=ROLES, default='student')

    def formatted_role(self) -> str:
        return ROLES.get(self.role, "???")

    def role_color(self) -> str:
        return ROLE_COLORS.get(self.role, "danger")


class RoleRequest(models.Model):
    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE)
    role_name = models.CharField(max_length=20, choices=ROLES)
