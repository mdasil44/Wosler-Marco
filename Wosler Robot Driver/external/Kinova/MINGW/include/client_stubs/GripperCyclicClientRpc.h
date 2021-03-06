#ifndef __GRIPPERCYCLICCLIENT_H__
#define __GRIPPERCYCLICCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "GripperCyclic.pb.h"

#include "Frame.h"
#include "IRouterClient.h"
#include "NotificationHandler.h"

#if __cplusplus >= 201402L
#define DEPRECATED [[ deprecated ]]
#define DEPRECATED_MSG(msg) [[ deprecated(msg) ]]
#elif defined(__GNUC__)
#define DEPRECATED __attribute__ ((deprecated))
#define DEPRECATED_MSG(msg) __attribute__ ((deprecated(msg)))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#define DEPRECATED_MSG(msg) __declspec(deprecated(msg))
#else
#define DEPRECATED 
#define DEPRECATED_MSG 
#endif
namespace Kinova
{
namespace Api
{
	namespace GripperCyclic
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidRefresh = 0x110001,
			eUidRefreshCommand = 0x110002,
			eUidRefreshFeedback = 0x110003,
			eUidRefreshCustomData = 0x110004,
		};
		
		class GripperCyclicClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdGripperCyclic;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			GripperCyclicClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			Feedback Refresh(const Command& command, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Refresh_callback(const Command& command, std::function< void (const Error&, const Feedback&) > callback, uint32_t deviceId = 0);
			std::future<Feedback> Refresh_async(const Command& command, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void RefreshCommand(const Command& command, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void RefreshCommand_callback(const Command& command, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> RefreshCommand_async(const Command& command, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Feedback RefreshFeedback(const MessageId& messageid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void RefreshFeedback_callback(const MessageId& messageid, std::function< void (const Error&, const Feedback&) > callback, uint32_t deviceId = 0);
			std::future<Feedback> RefreshFeedback_async(const MessageId& messageid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CustomData RefreshCustomData(const MessageId& messageid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void RefreshCustomData_callback(const MessageId& messageid, std::function< void (const Error&, const CustomData&) > callback, uint32_t deviceId = 0);
			std::future<CustomData> RefreshCustomData_async(const MessageId& messageid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif