from django.core.management.base import BaseCommand
from accounts.models import CustomUser


class Command(BaseCommand):
    help = "Delete a user account"

    def add_arguments(self, parser):
        parser.add_argument("--username", required=True)

    def handle(self, *args, **options):
        username = options["username"]

        user = CustomUser.objects.filter(username=username).first()

        if user:
            confirmation = input(f"Are you really sure you wanna delete '{user.username}'? [y/n]:")
            if confirmation.lower() == "y"  or "yes":
                user.delete()
                self.stdout.write(self.style.SUCCESS(f"User has unfortunately been deleted"))
            else:
                self.stdout.write(self.style.ERROR(f"Delete operation cancelled, '{user.username}' is saved"))
        else:
            self.stdout.write(
                self.style.ERROR(f"'{username}' does not exist")
            )
        
