"""
Test script for modules endpoint functionality
"""

try:
    from modules_endpoint import modules_db, CurriculumModule

    print(f"Successfully imported modules_endpoint")
    print(f"Total modules in database: {len(modules_db)}")
    
    for module in modules_db:
        print(f"- {module.id}: {module.title}")
        
    # Test the specific module endpoint function
    from modules_endpoint import get_module
    import asyncio
    
    # Create an event loop to run the async function
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        module = loop.run_until_complete(get_module('mod-ros2-001'))
        print(f"Specific module retrieval test passed!")
        print(f"Module title: {module.title}")
    except Exception as e:
        print(f"Error with async function: {e}")
    
    loop.close()

except ImportError as e:
    print(f"Import error: {e}")
    import traceback
    traceback.print_exc()

except Exception as e:
    print(f"General error: {e}")
    import traceback
    traceback.print_exc()