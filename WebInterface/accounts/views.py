from django.contrib import messages
from django.contrib.auth import login, logout
from django.contrib.auth.decorators import login_required
from django.contrib.auth.forms import AuthenticationForm
from django.shortcuts import render, redirect

from .models import CustomUser


# Handles storing registration info and sending new user to dashboard page
def register(request):
    if request.method == "POST":
        username = request.POST['username']
        password = request.POST['password']
        role = request.POST['role']

        if CustomUser.objects.filter(username=username).exists():
            messages.error(request, 'Username is already taken!', extra_tags='danger')
            context = {"prefillUser": username, "prefillPassword": password}
            return render(request, 'register.html', context)

        user = CustomUser.objects.create_user(username=username, password=password, role=role)
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


def demo_login(request):
    from accounts.models import CustomUser
    try:
        demo_user = CustomUser.objects.get(username='showcase', is_viewer=True)
        login(request, demo_user, backend='django.contrib.auth.backends.ModelBackend')
        return redirect('experiment_run_dynamic', experiment_name='CartControl')
    except CustomUser.DoesNotExist:
        return redirect('login')
