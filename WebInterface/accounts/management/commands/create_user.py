from django.core.management.base import BaseCommand
from accounts.models import CustomUser


class Command(BaseCommand):

    help = "Create New User Account"
    

    def add_arguments(self, parser):
        parser.add_argument("--username", required=True)
        parser.add_argument("--email", required=True)
        parser.add_argument("--password", required=True)
        parser.add_argument("--role", choices=["view_only", "student", "instructor"], default="student")

    

    def handle(self, *args, **options):
        username = options["username"]
        email = options["email"]
        password = options["password"]
        role = options["role"]

        if CustomUser.objects.filter(username=username).exists():
            self.stdout.write(
                self.style.ERROR(f"User '{username}' already exists :( <--sadface. ")
            )
            return
        

        user = CustomUser.objects.create_user(
            username=username,
            email=email,
            password=password,
            role=role
        )

        self.stdout.write(
            self.style.SUCCESS(f"Created user '{user.username}' with role '{user.role}' :) <--happy face.")
        )