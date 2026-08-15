
from django.shortcuts import render, redirect
from django.http import JsonResponse
from django.views.decorators.http import require_http_methods
from django.contrib.auth.decorators import login_required
from django.contrib import messages as django_messages
from .models import Command, Message
from .mqtt_utils import send_command
import json
from .models import ExperimentSession
from functools import wraps

#View for managing control lock for the experiment, ensuring only one user can control at a time, with a timeout mechanism
from django.utils import timezone
from .models import ControlLock

LOCK_TIMEOUT = 60 #seconds
from django.db import transaction

@login_required
def acquire_lock(request):
    if not request.session.session_key:
        request.session.save()

    # Viewer accounts can never acquire control
    if request.user.is_viewer:
        return JsonResponse({'status': 'viewer'})

    #ensure 2 users can't acquire at the same time
    with transaction.atomic():
        active = ControlLock.get_active(LOCK_TIMEOUT)
        if active:
            if active.session_key == request.session.session_key:
                return JsonResponse({'status': 'already_yours'})
            return JsonResponse({'status': 'locked', 'by': active.user})

        # No active lock — clear any stale ones and acquire atomically
        ControlLock.objects.all().delete()
        ControlLock.objects.create(
            session_key=request.session.session_key,
            user=str(request.user)
        )
    return JsonResponse({'status': 'acquired'})

@login_required
def release_lock(request):
    ControlLock.objects.filter(session_key=request.session.session_key).delete()
    return JsonResponse({'status': 'released'})

@login_required
def heartbeat(request):
    lock = ControlLock.objects.filter(session_key=request.session.session_key).first()
    if lock:
        lock.last_heartbeat = timezone.now()
        lock.save()
        return JsonResponse({'status': 'ok'})
    return JsonResponse({'status': 'lost'})  # Tell client they lost the lock

@login_required
def lock_status(request):
    active = ControlLock.get_active(LOCK_TIMEOUT)
    if not active:
        return JsonResponse({'locked': False})
    is_me = active.session_key == request.session.session_key
    return JsonResponse({'locked': True, 'is_me': is_me})

def requires_control_lock(view_func):
    @wraps(view_func)
    def wrapper(request, *args, **kwargs):
        lock = ControlLock.objects.filter(
            session_key=request.session.session_key
        ).first()
        if not lock:
            return JsonResponse({'error': 'Another user is currently controlling the experiment'}, status=403)
        # Refresh heartbeat on every action
        lock.last_heartbeat = timezone.now()
        lock.save()
        return view_func(request, *args, **kwargs)
    return wrapper


# Experiment configuration - default parameters for each experiment
EXPERIMENT_DEFAULTS = {
    'SwingHoldPendulum': {
            'parameters': {
                'PID_D': {
                    'proportional': {'value': 70, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.5, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 4.0,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_D1': {
                    'proportional': {'value': 25.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.2, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 1.5,  'editable': True, 'unit': 'kg⋅s'},
                },
            }
    },
    
    'CartControl': {
            'parameters': {
                'PID_D': {
                    'proportional': {'value': 27.8, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 50.0, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 3.9,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_D1': {
                    'proportional': {'value': 27.8, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 50.0, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 3.9,  'editable': True, 'unit': 'kg⋅s'},
                },
            }
    },

    'TestCartPend1': {
            'parameters': {
                'kp': {'value': 27.8,'editable': True, 'unit': 'unit'},
                'ki': {'value': 50.0, 'editable': True, 'unit': 'unit'},
                'kd': {'value': 3.9,  'editable': True, 'unit': 'unit'},
            }
    },

    'CraneIdent': {
            'parameters': {

        }
    },

    'CraneStab': {
            'parameters': {
                'PID_D' :{
                    'proportional': {'value': 8.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.3, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 4.0,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_D1' :{
                    'proportional': {'value': 5.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.1, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 0.2,  'editable': True, 'unit': 'kg⋅s'},
                }
        },
    },

    'InvPendIdent': {
            'parameters': {
                'PID' : {
                    'proportional': {'value': 20.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.1, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 2.0,  'editable': True, 'unit': 'kg⋅s'},  
                },
                'PID_D' : {
                    'proportional': {'value': 7.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.1, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 2.0,  'editable': True, 'unit': 'kg⋅s'},  
                },
                'PID_1' : {
                    'proportional': {'value': 7.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.1, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 2.0,  'editable': True, 'unit': 'kg⋅s'},  
                },
                'PID_D1' : {
                    'proportional': {'value': 20.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.1, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 2.0,  'editable': True, 'unit': 'kg⋅s'},  
                },
            }
    },

    'PendStabPD': {
            'parameters': {
                'PID_1' : {
                    'proportional': {'value': 8.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.3, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 4.0,  'editable': True, 'unit': 'kg⋅s'},  
                },
                'PID_D1' : {
                    'proportional': {'value': 25.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.1, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 1.8,  'editable': True, 'unit': 'kg⋅s'},  
                },
            }
    },

    'PendSwingUp': {
            'parameters': {
                'voltage amplitude': {'value': 0.2, 'editable': True, 'unit': 'V'},
                'angle comparison value': {'value': 1.0, 'editable': True, 'unit': 'rad'}
            }
    },
    'PendulumFriction': {
            'parameters': {
            }
    },

    'SwingHoldPendulumExtra': {
            'parameters': {
                'PID_D' :{
                    'proportional': {'value': 7.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.5, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 4.0,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_D1' :{
                    'proportional': {'value': 25.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.2, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 1.5,  'editable': True, 'unit': 'kg⋅s'},
                },
        }
    },

    'UpDownPendulum': {
            'parameters': {
                'PID_DIPS' :{
                    'proportional': {'value': 7.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.5, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 3.0,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_D1IPS' :{
                    'proportional': {'value': 25.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.2, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 2.0,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_DCC' :{
                    'proportional': {'value': 7.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 2.0, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 1.0,  'editable': True, 'unit': 'kg⋅s'},
                },
                'PID_D1CC' :{
                    'proportional': {'value': 4.0, 'editable': True, 'unit': 'kg'},
                    'integral':     {'value': 0.0, 'editable': True, 'unit': 'kg/s'},
                    'derivative':   {'value': 0.1,  'editable': True, 'unit': 'kg⋅s'},
                },

            }
    },
    'PendulumTest': {},
    'InvPendulumStream': {
        'parameters': {}
    },
    
}



@login_required
def dashboard(request):
    """
    Main dashboard - shows experiment selection grid
    """
    
    return render(request, 'MatlabApp/dashboard.html')


@login_required
def experiment_run_dynamic(request, experiment_name):
    """
    Display the experiment run page with parameters and controls
    """
    # Check if experiment exists
    if experiment_name not in EXPERIMENT_DEFAULTS:
        django_messages.error(request, f'Experiment "{experiment_name}" not found')
        return redirect('dashboard')
    
    # Get parameters for this specific experiment
    experiment_config = EXPERIMENT_DEFAULTS[experiment_name]

    # Handle both old format (just values) and new format (with metadata)
    if 'parameters' in experiment_config:
        parameters = experiment_config['parameters']
    else:
        # Old format - treat all as editable for backwards compatibility
        parameters = {k: {'value': v, 'editable': True, 'unit': ''} 
                     for k, v in experiment_config.items()}
    
    # Get recent activity
    recent_commands = Command.objects.all()[:10]
    raspi_messages = Message.objects.all()[:10]
    
    context = {
        'experiment_name': experiment_name,
        'parameters': parameters,  # PASS PARAMETERS TO TEMPLATE
        'recent_commands': recent_commands,
        'raspi_messages': raspi_messages,
        'is_instructor': request.user.is_instructor,
    }
    
    return render(request, 'MatlabApp/experiment_run_dynamic.html', context)

@login_required
def estop(request):
    # Only instructors can use estop
    if not request.user.is_instructor:
        return JsonResponse({'error': 'Not authorized'}, status=403)

    # Clear whoever currently holds the lock
    ControlLock.objects.all().delete()

    # Give the instructor control
    if not request.session.session_key:
        request.session.save()

    ControlLock.objects.create(
        session_key=request.session.session_key,
        user=str(request.user)
    )

    return JsonResponse({'status': 'estop_ok'})

@login_required
@requires_control_lock
@require_http_methods(["POST"])
def send_experiment_command(request, experiment_name):
    try:
        data = json.loads(request.body)
        command = data.get('command')

        if not command:
            return JsonResponse({'success': False, 'error': 'No command provided'})

        payload = {
            "command": command,
            "experiment": experiment_name
        }

        # If START, attach parameters from request or fall back to saved/defaults
        if command == "sta":
            params_from_request = data.get('parameters', {})
            
            # Only build nested structure for experiments that use grouped PID params
            nested_experiments = []
            
            if params_from_request:
                if experiment_name in nested_experiments:
                    nested = {}
                    for key, value in params_from_request.items():
                        parts = key.rsplit('_', 1)
                        if len(parts) == 2:
                            group, param = parts
                            if group not in nested:
                                nested[group] = {}
                            nested[group][param] = value
                        else:
                            nested[key] = value
                    payload["parameters"] = nested
                else:
                    # All other experiments — send flat as before
                    payload["parameters"] = params_from_request
            else:
                # Fall back to last saved session parameters
                try:
                    session = ExperimentSession.objects.get(
                        experiment_name=experiment_name,
                    )
                    payload["parameters"] = session.parameters
                except ExperimentSession.DoesNotExist:
                    # Fall back to defaults
                    exp_params = EXPERIMENT_DEFAULTS[experiment_name].get('parameters', {})
                    if experiment_name in nested_experiments:
                        payload["parameters"] = {
                            group: {
                                param: config['value']
                                for param, config in params.items()
                            }
                            for group, params in exp_params.items()
                        }
                    else:
                        payload["parameters"] = {
                            k: v["value"] for k, v in exp_params.items()
                        }

        cmd_obj = Command.objects.create(
            command=command,
        )

        success = send_command(payload)

        cmd_obj.success = success
        cmd_obj.save()

        return JsonResponse({
            'success': success,
            'payload': payload
        })

    except Exception as e:
        return JsonResponse({
            'success': False,
            'error': str(e)
        })

@login_required
@requires_control_lock
@require_http_methods(["POST"])
def update_experiment_params(request, experiment_name):
    try:
        data = json.loads(request.body)
        parameters = data.get('parameters', {})

        if experiment_name not in EXPERIMENT_DEFAULTS:
            return JsonResponse({
                'success': False,
                'error': f'Experiment "{experiment_name}" not found'
            })

        # Save or update parameters in DB
        session, created = ExperimentSession.objects.update_or_create(
            experiment_name=experiment_name,
            user=request.user,
            defaults={'parameters': parameters}
        )

        # Log update action
        Command.objects.create(
            command=f'Parameters updated for {experiment_name}',
            #user=request.user,
        )

        return JsonResponse({
            'success': True,
            'message': 'Parameters saved successfully',
            'parameters': parameters
        })

    except Exception as e:
        return JsonResponse({
            'success': False,
            'error': str(e)
        })


@login_required
@requires_control_lock
def get_experiment_defaults(request, experiment_name):
 
    if experiment_name not in EXPERIMENT_DEFAULTS:
        return JsonResponse({
            'success': False,
            'error': f'Experiment "{experiment_name}" not found'
        })
    
    return JsonResponse({
        'success': True,
        'experiment': experiment_name,
        'defaults': EXPERIMENT_DEFAULTS[experiment_name]
    })


@login_required
def data_history(request):
    """Display all historical commands and messages"""
    
    commands = Command.objects.all().order_by('-timestamp')
    raspi_messages = Message.objects.all().order_by('-timestamp')
    
    return render(request, 'MatlabApp/data_history.html', {
        'commands': commands,
        'raspi_messages': raspi_messages,
    })

@login_required
def pendulum_stream(request):
    """New Camera stream for testing speed"""
    return render(request, 'MatlabApp/pendulum_stream.html', {})
