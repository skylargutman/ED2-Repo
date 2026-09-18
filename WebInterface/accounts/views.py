import json
import os
import secrets

from django.contrib import messages
from django.contrib.auth import login, logout
from django.contrib.auth.decorators import login_required
from django.contrib.auth.forms import AuthenticationForm
from django.http import HttpResponse
from django.shortcuts import render, redirect
from django.views.decorators.csrf import csrf_exempt

from .models import CustomUser, RoleRequest

EMAIL_VERIFICATION_SECRET = os.environ.get("EMAIL_VERIFICATION_SECRET", None)
EMAIL_VERIFICATION_LINK = os.environ.get("EMAIL_VERIFICATION_LINK", None)
EMAIL_VERIFICATION_ENABLED = EMAIL_VERIFICATION_SECRET and EMAIL_VERIFICATION_LINK


# Handles storing registration info and sending new user to dashboard page
def register(request):
    if request.method == "POST":
        username = request.POST['username']
        password = request.POST['password']
        email = request.POST['email']
        requested_role = request.POST['role']

        if CustomUser.objects.filter(username=username).exists():
            messages.error(request, 'Username is already taken!', extra_tags='danger')
            context = {"prefillUser": username, "prefillEmail": email, "prefillPassword": password}
            return render(request, 'register.html', context)

        if EMAIL_VERIFICATION_ENABLED:
            user = CustomUser.objects.create_user(username=username, password=password, email=email, role='view_only',
                                                  is_viewer=True)
            RoleRequest.objects.create(user=user, role_name=requested_role)
        else:
            user = CustomUser.objects.create_user(username=username, password=password, email=email,
                                                  role=requested_role)

        messages.success(request, "Account created successfully!")
        login(request, user)
        return redirect('dashboard')
    return render(request, 'register.html')


# Check existing user data and login w/ Django's built-in login
def login_view(request):
    if request.method == "POST":
        form = AuthenticationForm(request, data=request.POST)
        if form.is_valid():
            user = form.get_user()
            login(request, user)
            return redirect('dashboard')
    else:
        form = AuthenticationForm()
    return render(request, 'login.html', {'form': form})


# Django built-in logout
@login_required
def logout_view(request):
    # Clear any active control locks for this user when they log out
    from MatlabApp.models import ControlLock
    ControlLock.objects.filter(
        session_key=request.session.session_key
    ).delete()

    logout(request)
    return redirect('login')


@csrf_exempt
def verify_email_view(request):
    if not request.method == "POST":
        return HttpResponse(status=400)
    if request.content_type != "application/json":
        return HttpResponse(status=415)
    if not EMAIL_VERIFICATION_ENABLED:
        return HttpResponse(status=400)

    from accounts.models import CustomUser
    data = json.loads(request.body)
    secret_key = data["secret"]
    email = data["email"]
    username = data["username"]

    if not secrets.compare_digest(secret_key, EMAIL_VERIFICATION_SECRET):
        return HttpResponse(status=400)

    query = CustomUser.objects.filter(username=username, email=email, role="view_only")
    if not query.exists():
        return HttpResponse(status=404)

    user = query.get()
    print("Received email verification for", user)
    user.is_viewer = False

    roleRequestQuery = RoleRequest.objects.filter(user=user)
    if roleRequestQuery.exists():
        roleRequest = roleRequestQuery.get()
        print("Granting role", roleRequest.role_name, "for", user)
        user.role = roleRequest.role_name
        print("Removing role request #", roleRequest.id, sep="")
        roleRequest.delete()

    user.save()
    return HttpResponse(status=200)


def demo_login(request):
    from accounts.models import CustomUser
    try:
        demo_user = CustomUser.objects.get(username='showcase', is_viewer=True, role='view_only')
        login(request, demo_user, backend='django.contrib.auth.backends.ModelBackend')
        return redirect('experiment_run_dynamic', experiment_name='CartControl')
    except CustomUser.DoesNotExist:
        return redirect('login')
