from django.urls import path
from accounts import views as accounts_views
from . import views

# NOTE: no 'admin/' route here. EGNSite/urls.py already registers
# admin.site.urls, and registering it twice makes Django emit
#   urls.W005: URL namespace 'admin' isn't unique
# and can break reverse() for admin URLs.
urlpatterns = [
    # Authentication URLs
    path('login/', accounts_views.login_view, name='login'),
    path('register/', accounts_views.register, name='register'),
    path('logout/', accounts_views.logout_view, name='logout'),
    
    # Main App URLs
    path('dashboard/', views.dashboard, name='dashboard'),
    path('data_history/', views.data_history, name='data'),

    #Camera Stream
    path('live/', views.pendulum_stream, name='pendulum_stream'),

    #experimets
    path('experiment/<str:experiment_name>/', views.experiment_run_dynamic, name='experiment_run_dynamic'),
    path('experiment/<str:experiment_name>/command/', views.send_experiment_command, name='send_experiment_command'),
    path('experiment/<str:experiment_name>/params/', views.update_experiment_params, name='update_experiment_params'),
    path('experiment/<str:experiment_name>/defaults/', views.get_experiment_defaults, name='get_experiment_defaults'),

    #Emergency stop
    path('control/estop/', views.estop, name='estop'),

    #Control Lock URLs
    path('control/acquire/', views.acquire_lock, name='acquire_lock'),
    path('control/release/', views.release_lock, name='release_lock'),
    path('control/heartbeat/', views.heartbeat, name='heartbeat'),
    path('control/status/', views.lock_status, name='lock_status'),

    #demo login
    path('demo/', accounts_views.demo_login, name='demo_login'),
]
