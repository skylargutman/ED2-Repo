from django.core.management.base import BaseCommand
from accounts.models import CustomUser
import os
import sys
HEADER_FORMAT = "{:<20} | {:<30} | {:<12}"
ROW_FORMAT = "{:<20} | {:<30} | {:<12}"


class Command(BaseCommand):
    help: "Lookup Specified Users"


    def handle(self, *args, **options):
        users = CustomUser.objects.all()

        status = True
        while status:
            os.system("cls" if os.name =="nt" else "clear")
            self._display_users(users)

            users = self._field_filter(users)
            if users == "exit":
                status=False
                break
            if users == "reset":
                users=CustomUser.objects.all()
                continue
            



    def _field_filter(self, users):
        typeF = input("Filter By\n(U)sername\n(E)mail\n(R)ole\n(N)one\n(Re)set\nE(X)it\n>>")
        
        if typeF.lower() in ["username", "u"]:
            filter = input(">>")
            users = self._match_type_filters(typeF, filter, users)
            return users
        if typeF.lower() in ["email", "e"]:
            filter = input(">>")
            users = self._match_type_filters(typeF, filter, users)
            return users
        if typeF.lower() in ["role", "r"]:
            filter = input(">>")
            users = self._match_type_filters(typeF, filter, users)
            return users
        if typeF.lower() in ["exit", "x"]:
            return "exit"
        if typeF.lower() in ["reset", "re"]:
            return "reset"
        else:
            return "reset"

    
    def _match_type_filters(self, typeF, filter, users):
        self.stdout.write("-"*20)
        match_type_map = {
            "co": "__icontains",
            "sw": "__startswith",
            "ew": "__endswith"
        }
        match_type_input = input("Match Type\nco: contains\nsw: starts with\new: ends with\n(N)one\n>>")
        match_type = match_type_map.get(match_type_input.lower())

        if match_type_input.lower() in ["co", "sw", "ew"]:
            if typeF.lower() in ["username", "u"]:
                argumentKey = 'username' + match_type
                query_arguments = {argumentKey: filter}
                users = CustomUser.objects.filter(**query_arguments)
                return users
            if typeF.lower() in ["email", "e"]:
                argumentKey = 'email' + match_type
                query_arguments = {argumentKey: filter}
                users = CustomUser.objects.filter(**query_arguments)
                return users
            if typeF.lower() in ["role", "r"]:
                argumentKey = 'role' + match_type
                query_arguments = {argumentKey: filter}
                users = CustomUser.objects.filter(**query_arguments)
                return users
        else:
            if typeF.lower() in ["username", "u"]:
                argumentKey = 'username'
            if typeF.lower() in ["email", "e"]:
                argumentKey= 'email'
            if typeF.lower() in ["role", "r"]:
                argumentKey = 'role'
            
            query_arguments = {argumentKey: filter}
            users = CustomUser.objects.filter(**query_arguments)
            return users
            

    def _display_users(self, users):
        self.stdout.write(self.style.SUCCESS(
            HEADER_FORMAT.format("Username", "Email", "Role")
        ))

        self.stdout.write("-"*78)

        for user in users:
            self.stdout.write(ROW_FORMAT.format(
                user.username,
                user.email,
                user.role
            ))

        self.stdout.write("\n"+ "-"*78)

        