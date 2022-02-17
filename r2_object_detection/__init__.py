try:
    from . import inference, utility_constants, edgedetection
except ImportError:
    try:
        import inference
        import utility_constants
        import edgedetection
    except ImportError as e:
        print('Warning: failed to import a dependency. See the below error')
        print(e)
