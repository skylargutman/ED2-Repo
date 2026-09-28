from django.core.management.base import BaseCommand
from accounts.models import CustomUser


class Command(BaseCommand):
    help = "Modify User Info"

    def add_arguments(self, parser):
        parser.add_argument("--username", required=True)
        parser.add_argument("--newUsername", required=False)
        parser.add_argument("--newEmail", required=False)
        parser.add_argument("--newPassword", required=False)
        parser.add_argument("--newRole", choices = ["view_only", "student", "instructor"], required=False)

    def handle(self, *args, **options):
        currentUsername = options["username"]
        newUsername = options["newUsername"]
        newEmail = options["newEmail"]
        newPassword = options["newPassword"]
        newRole = options["newRole"]

        user = CustomUser.objects.filter(username=currentUsername).first()

        if user is None:
            self.stdout.write(self.style.ERROR("Uh Oh, this user doesnt exist"))
            return


        if newUsername is not None:
            user.username = newUsername
            self.stdout.write(self.style.SUCCESS(f"Username set to Update ^_^"))

        if newEmail is not None:
            user.email = newEmail
            self.stdout.write(self.style.SUCCESS(f"Email set to Update ^_^"))

        if newPassword is not None:
            user.set_password(newPassword)
            self.stdout.write(self.style.SUCCESS(f"Password set to Update ^_^"))

        if newRole is not None:
            user.role = newRole
            self.stdout.write(self.style.SUCCESS(f"Role set to Update ^_^"))

        try:
            user.save()
            self.stdout.write(self.style.SUCCESS(f"User Successfully Updated. They're a brand new person now."))
        except Exception as e:
            self.stdout.write(self.style.ERROR(f"Failed to update user: {e}"))


        
                